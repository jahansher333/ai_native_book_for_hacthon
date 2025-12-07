import os
from agents import Agent, Runner, function_tool, AsyncOpenAI, OpenAIChatCompletionsModel, set_tracing_disabled
from dotenv import load_dotenv


load_dotenv(override=True)
set_tracing_disabled(True)

gemini_api_key = os.getenv("GEMINI_API_KEY")
base_url = os.getenv("BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")    


# Initialize Gemini via OpenAI-compatible endpoint
external_provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url=base_url
)

# Create Gemini model instance
gemini_model = OpenAIChatCompletionsModel(
    model="gemini-2.0-flash",
    openai_client=external_provider
)
rag_agent = Agent(
    name="RAGAgent",
    instructions="You are an intelligent agent that uses retrieval-augmented generation to answer questions about the Physical AI & Humanoid Robotics textbook. Always use the provided textbook content to inform your answers.",
    model=gemini_model
)

run = Runner.run_sync(
    rag_agent,
    "What are the advantages of using NVIDIA Isaac Sim for robotics simulation compared to traditional simulators like Gazebo?"
)

print(run.final_output)

