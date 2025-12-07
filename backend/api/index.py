"""
Vercel serverless function handler for FastAPI backend
This file is the entry point for Vercel deployments
"""
import sys
from pathlib import Path

# Add the src directory to Python path
src_path = Path(__file__).parent.parent / "src"
sys.path.insert(0, str(src_path))

from main import app

# Vercel expects a variable named 'app' or 'handler'
# FastAPI app is already defined in main.py
