"""Test if query.py can import the LiteLLM RAG agent"""
import sys
import os

# Set up paths like FastAPI does
backend_dir = os.path.dirname(__file__)
sys.path.insert(0, backend_dir)

print("Testing query.py import...")
print(f"Python path[0]: {sys.path[0]}")

try:
    # This will trigger the import chain
    from src.api import query
    print("SUCCESS: query module imported")
    print(f"run_rag_agent function: {query.run_rag_agent}")
    print(f"Module: {query.run_rag_agent.__module__}")
except Exception as e:
    print(f"FAILED: {e}")
    import traceback
    traceback.print_exc()
