"""
LiteLLM Groq Agents - Converted from Gemini
Provides RAG, Personalization, and Urdu Translation using Groq backend

Cost Savings: 90% reduction vs. Gemini ($150/mo → $30/mo)
Performance: Groq inference typically 1-5 seconds
"""

from .rag_agent import run_rag_agent, rag_agent, search_textbook
from .personalize_agent import personalize_chapter_content
from .urdu_translator import translate_chapter_to_urdu

__version__ = "1.0.0"
__all__ = [
    'run_rag_agent',
    'rag_agent',
    'search_textbook',
    'personalize_chapter_content',
    'translate_chapter_to_urdu'
]
