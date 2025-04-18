#!/usr/bin/env python3

import rospy
import time
from collections import deque
from sensor_msgs.msg import Image
from PIL import Image as PILImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import base64
import openai
import json
import numpy as np
from utils import get_openai_client
from env import MODEL
import cv2
import logging
import logging.config
from uuid import uuid4
from prompts import prompt_with_history
import threading
import os

# logging.config.dictConfig(
#     {
#         'version': 1,
#         # 'disable_existing_loggers': True
#     }
# )
# logger = logging.getLogger('webcam')

class Camera():
    def __init__(self, stack_size, interval):
        self.stack_size = stack_size
        self.interval = interval

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.last_added_time = time.perf_counter()

    def reset(self):
        self.frame_stack = deque(maxlen=self.stack_size)

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Encode image to base64 if sending image content (optional)
            current_time = time.perf_counter()
            if current_time - self.last_added_time >= self.interval:
                self.frame_stack.append(frame)
                self.last_added_time = current_time

        except Exception as e:
            rospy.logerr(f"Error processing image or GPT query: {e}")

class QueryGPT(threading.Thread):
    def __init__(self, cam: Camera=None, stack_size=None, logger=None, episode_path=None, create_prompt=prompt_with_history):
        super(QueryGPT, self).__init__()
        self.logger = logger
        self.stack_size = stack_size
        self.episode_path = episode_path
        self.client = get_openai_client()
        self.cam = cam
        self.create_prompt_fn = create_prompt
        self.reset()

        rospy.init_node('gpt_node')
        self.gpt_pub = rospy.Publisher('/gpt_action', String, queue_size=1)
    
    def run(self):
        self.running = True
        self.query_gpt()
    
    def reset(self):
        self.history = []
        self.running = False
        if self.cam:
            self.cam.reset()

    def create_prompt(self, image_path=None, observation=None):
        prompt = self.create_prompt_fn(self.stack_size)
        messages = [
            {"role": "system", "content": prompt["system"]},
        ]
        # messages += [{"role": "assistant", "content": prompt["examples"]}]
        messages += self.history

        message_text = prompt["user"]
        if observation is not None:
            message_text += observation
        messages.append(
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": message_text,
                    },
                ],
            },
        )
        if image_path is not None:
            messages[-1]["content"].append(
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{self.encode_image(image_path)}",
                        "detail": "high",
                    },
                },
            )
        return messages


    def log_prompt(self, prompt, image_path):
        temp = prompt[-1]["content"][-1]["image_url"]["url"]
        prompt[-1]["content"][-1]["image_url"]["url"] = image_path
        self.logger.info(prompt)
        prompt[-1]["content"][-1]["image_url"]["url"] = temp


    def call_gpt(self, prompt):
        response = self.client.chat.completions.create(
            model=MODEL,
            messages=prompt,
            max_tokens=10000,
            temperature=1.0,
            top_p=1.0,
        )
        return response.choices[0].message

    def encode_image(self, image):
        if type(image) is str:
            with open(image, "rb") as image_file:
                return base64.b64encode(image_file.read()).decode("utf-8")
        else:
            return base64.b64encode(image.to_bytes()).decode("utf-8")

    def stack_frames(self, frames):
        # Resize and stack frames horizontally
        # resized_frames = [cv2.resize(frame, (200, 150)) for frame in frames]
        stacked_image = np.hstack(frames)
        return PILImage.fromarray(cv2.cvtColor(stacked_image, cv2.COLOR_BGR2RGB))

    def query_gpt_with_cam_once(self, i):
        frame_stack = self.cam.frame_stack
        if frame_stack and len(frame_stack) == self.stack_size:
            stacked_image = self.stack_frames(frame_stack)
            image_path = f"{self.episode_path}/image_stack_{i}.png"
            stacked_image.save(image_path)

            prompt, response, action, time = self.query_gpt_once(image_path)
            if self.logger:
                self.log_prompt(prompt, image_path)
                self.logger.info(
                    f"GPT-4 Response: {action}, time_elapsed: {'%.3f'%(time)}s"
                )

            return response
    
    def query_gpt_once(self, image_path=None, observation=None):
        prompt = self.create_prompt(image_path, observation)
        start_time = time.perf_counter()
        gpt_response = self.call_gpt(prompt)
        end_time = time.perf_counter()
        response = {
                "role": "assistant",
                "content": [
                    {
                        "type": "text",
                        "text": gpt_response.content,
                    }
                ],
            }
        self.history.append(response)
        return prompt, response, gpt_response.content, end_time - start_time

    def format_analysis(self, state):
        text = state['content'][0]['text']
        formatted_text = json.loads(text[text.find('{'):text.find('}') + 1])
        return formatted_text

    def query_gpt(self):
        i = 0
        while self.running and not rospy.is_shutdown():
            response = self.query_gpt_with_cam_once(i)
            if response is not None:
                i += 1
                action = self.format_analysis(self.history[-1])['action']
                print(action)
                self.gpt_pub.publish(str(action))

    def test(self, observations):
        actions = []
        for observation in observations:
            _, _, action, _ = self.query_gpt_once(observation=observation)
            actions.append(action)
        return actions
    
if __name__ == "__main__":
    episode_id = str(uuid4())[:10]
    episode_path = f'/home/mirrorbot/catkin_ws/src/mirror_bot/src/episodes/episode_{episode_id}'
    os.makedirs(episode_path, exist_ok=True)

    stack_size = 10
    time_per_stack = 2.0
    interval = time_per_stack / stack_size
    camera = Camera(stack_size, interval)
    prompter = QueryGPT(camera, stack_size, episode_path=episode_path)
    prompter.start()

    rospy.spin()