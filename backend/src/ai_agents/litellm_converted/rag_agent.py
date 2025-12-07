"""
RAG Agent using LiteLLM with Groq backend
Migrated from Gemini to Groq for cost optimization (90% reduction)
"""
import asyncio
import os
import re
from typing import Dict, List, Optional

# Import openai-agents BEFORE local imports to avoid name collision
import sys
_original_path = sys.path[:]
sys.path = [p for p in sys.path if 'backend' not in p and 'src' not in p.lower()]
try:
    from agents import Agent, Runner, function_tool  # openai-agents package
    from agents.extensions.models.litellm_model import LitellmModel  # openai-agents extension
finally:
    sys.path = _original_path

from dotenv import load_dotenv

# Import existing services - use try/except for both import styles
try:
    # Try relative imports first (FastAPI context)
    from ...services.embeddings import embedding_service
    from ...services.vector_store import vector_store_service
    from ...config import settings
except ImportError:
    # Fall back to absolute imports (standalone script context)
    from services.embeddings import embedding_service
    from services.vector_store import vector_store_service
    from config import settings

# Load environment
load_dotenv(override=True)
groq_api_key = os.getenv("GROQ_API_KEY")


@function_tool
async def search_textbook(query: str) -> str:
    """
    Search the Physical AI & Humanoid Robotics textbook for relevant content.

    This tool retrieves the most relevant sections from the textbook based on semantic similarity.
    Always use this tool to find information before answering questions.

    Args:
        query: The search query or question to find relevant textbook content for

    Returns:
        Formatted context with relevant textbook sections and their sources
    """
    try:
        # Embed the query
        query_embedding = await embedding_service.embed_query(query)

        # Search Qdrant for top-5 similar chunks
        results = await vector_store_service.search(
            query_embedding=query_embedding,
            limit=settings.top_k_results,
            score_threshold=settings.similarity_threshold
        )

        if not results:
            return "No relevant content found in the textbook for this query. The answer may not be available in the course materials."

        # Format results with sources
        context_parts = []
        for idx, result in enumerate(results, 1):
            payload = result['payload']
            score = result['score']

            context_parts.append(
                f"[Source {idx}] (Relevance: {score:.2f})\\n"
                f"Module: {payload['module']}\\n"
                f"Section: {payload['section']}\\n"
                f"Page: {payload['page']}\\n"
                f"Content: {payload['text']}\\n"
                f"---"
            )

        return "\\n\\n".join(context_parts)

    except Exception as e:
        return f"Error searching textbook: {str(e)}"


# Create Groq model instance
groq_model = LitellmModel(
    model="groq/llama-3.3-70b-versatile",  # Current Groq model
    api_key=groq_api_key
)

# Create RAG agent with Groq (T007, T008, T009)
rag_agent = Agent(
    name="TextbookAssistant",
    instructions="""You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.

CRITICAL RULES:
1. **Book-Only Retrieval**: Answer ONLY using information from the textbook. You MUST use the search_textbook function to find relevant content before answering any question.
2. **Always Cite Sources**: Include citations in the format [Source: Module - Section, Page X] for all factual claims.
3. **Refuse Non-Textbook Questions**: If asked about topics outside the textbook (weather, news, general knowledge), politely say: "I can only answer questions about the Physical AI & Humanoid Robotics textbook content. Please ask about ROS 2, Gazebo, Unity, Isaac Sim, Vision-Language-Action models, or hardware requirements."
4. **Selected Text Priority**: If the user provides selected text from the page, prioritize that context in your answer.
5. **Accuracy First**: Be concise but accurate. If the information is not in the search results, say so clearly.
6. **Emphasize Best Practices**: When relevant, emphasize sim-to-real workflows, edge deployment on Jetson devices, and the dangers of high-latency cloud control for robots.

ANSWER FORMAT:
- Start with a direct answer
- Provide details from the textbook
- End with proper citations
- Keep responses concise (2-4 paragraphs maximum)

Example citation format:
"ROS 2 uses DDS middleware for communication [Source: ROS 2 - Introduction, Page 5]"
""",
    tools=[search_textbook],
    model=groq_model
)


async def run_rag_agent(
    question: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None
) -> Dict:
    """
    Run the RAG agent to answer a question using textbook content (T010, T011)

    Args:
        question: User's question
        selected_text: Optional text selected by user on the page
        session_id: Optional session identifier

    Returns:
        Dict with answer, sources (citations), and confidence score
    """
    try:
        # Enhance question with selected text if provided
        if selected_text:
            enhanced_question = f"""The user has selected the following text from the page:

---SELECTED TEXT---
{selected_text[:2000]}
---END SELECTED TEXT---

User's question about this selection: {question}

Please answer the question using both the selected text context and relevant information from the textbook."""
        else:
            enhanced_question = question

        # Run the agent with 60-second timeout
        result = await asyncio.wait_for(
            Runner.run(rag_agent, enhanced_question),
            timeout=60.0
        )

        # Extract the final answer
        answer = result.final_output if hasattr(result, 'final_output') else str(result)

        # Extract citations from the answer using regex
        citations = extract_citations(answer)

        # Calculate confidence based on number of sources found
        confidence = min(0.9, 0.5 + (len(citations) * 0.1))

        return {
            "answer": answer,
            "citations": citations,
            "confidence": confidence
        }

    except asyncio.TimeoutError:
        return {
            "answer": "The query took too long (>60s). Try a simpler question or try again later.",
            "citations": [],
            "confidence": 0.0
        }
    except Exception as e:
        error_message = str(e)

        # Check for rate limit errors (429) - T011
        if "429" in error_message or "quota" in error_message.lower() or "rate" in error_message.lower():
            return {
                "answer": "⚠️ API rate limit exceeded. The Groq API has reached its quota limit. Please try again in a minute or contact the administrator to upgrade the API plan.",
                "citations": [],
                "confidence": 0.0
            }

        # Return generic error response
        return {
            "answer": f"I encountered an error while processing your question: {error_message}. Please try rephrasing your question or contact support if the issue persists.",
            "citations": [],
            "confidence": 0.0
        }


def extract_citations(answer: str) -> List[Dict]:
    """
    Extract citations from the agent's answer

    Args:
        answer: The agent's response text

    Returns:
        List of citation dictionaries
    """
    citations = []

    # Pattern to match citations like [Source: Module - Section, Page X]
    pattern = r'\[Source:\s*([^-]+?)\s*-\s*([^,]+?),\s*Page\s*(\d+)\]'
    matches = re.findall(pattern, answer)

    for match in matches:
        module, section, page = match
        citations.append({
            "module": module.strip(),
            "section": section.strip(),
            "page": int(page),
            "source_file": f"{module.lower().replace(' ', '-')}/{section.lower().replace(' ', '-')}.md",
            "chunk_id": "",  # Will be filled from search results
            "relevance_score": 0.85,  # Default score
            "text_snippet": ""  # Excerpt from the source
        })

    return citations


print("*** LiteLLM RAG agent loaded from ai_agents/litellm_converted/rag_agent.py ***")
__all__ = ['run_rag_agent', 'rag_agent', 'search_textbook']
