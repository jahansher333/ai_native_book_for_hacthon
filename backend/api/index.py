"""
Serverless function handler for FastAPI backend
This file is the entry point for Vercel deployments
For Railway deployment, use src/main.py directly with Procfile
"""
import sys
import os
from pathlib import Path

# Add the src directory to Python path
backend_root = Path(__file__).parent.parent
src_path = backend_root / "src"
sys.path.insert(0, str(backend_root))
sys.path.insert(0, str(src_path))

# Set working directory
os.chdir(str(backend_root))

# Import the FastAPI app
from src.main import app

# Vercel expects a variable named 'app' or 'handler'
handler = app
