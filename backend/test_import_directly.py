#!/usr/bin/env python
"""Test if the LiteLLM RAG agent can be imported directly"""
import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

print("Testing import...")
try:
    from ai_agents.litellm_converted.rag_agent import run_rag_agent
    print(f"✓ SUCCESS: Imported run_rag_agent from {run_rag_agent.__module__}")
    print(f"  Function: {run_rag_agent}")
except ImportError as e:
    print(f"✗ FAILED: {e}")
    import traceback
    traceback.print_exc()
