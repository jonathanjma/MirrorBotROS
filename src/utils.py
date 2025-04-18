from openai import OpenAI
from env import OAI_API_KEY, GEMINI_API_KEY

def get_openai_client():
    return OpenAI(
        api_key=OAI_API_KEY,
    )