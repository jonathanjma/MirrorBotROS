import pygame

def play_sound(file_path):
    # Initialize the mixer module
    pygame.mixer.init()
    # Load the sound file
    pygame.mixer.music.load(file_path)
    # Play the sound file
    pygame.mixer.music.play()

    print("Playing sound...")
    # Keep the program running until the sound finishes
    while pygame.mixer.music.get_busy():
        continue

# Specify the sound file path (e.g., an MP3 file)
sound_file = "/home/yyt/catkin_ws/src/mirror_bot/src/electronic-doorbell-262895.mp3"  # Replace with your file path
play_sound(sound_file)
