"""
Test script to verify LiteLLM agent imports work correctly
This mimics how FastAPI would import the agents
"""
import sys
import os

# Add backend/src to path (same as FastAPI does)
backend_src = os.path.join(os.path.dirname(__file__), 'src')
sys.path.insert(0, backend_src)

print(f"Python path: {sys.path[0]}")
print(f"Testing imports from: {backend_src}")
print()

try:
    print("1. Importing RAG agent...")
    from ai_agents.litellm_converted.rag_agent import run_rag_agent
    print("   SUCCESS: RAG agent imported")
except Exception as e:
    print(f"   FAILED: {e}")
    import traceback
    traceback.print_exc()

try:
    print("2. Importing Personalize agent...")
    from ai_agents.litellm_converted.personalize_agent import personalize_chapter_content
    print("   SUCCESS: Personalize agent imported")
except Exception as e:
    print(f"   FAILED: {e}")

try:
    print("3. Importing Urdu translator...")
    from ai_agents.litellm_converted.urdu_translator import translate_chapter_to_urdu
    print("   SUCCESS: Urdu translator imported")
except Exception as e:
    print(f"   FAILED: {e}")

print()
print("All imports successful!")
