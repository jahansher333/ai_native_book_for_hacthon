"""
RAG Agent using OpenAI Agents SDK with Gemini API
Implements intelligent retrieval-augmented generation for textbook queries
"""
import asyncio
import os
import re
from typing import Dict, List, Optional
from agents import Agent, Runner, function_tool, AsyncOpenAI, OpenAIChatCompletionsModel, set_tracing_disabled
from dotenv import load_dotenv

from ..config import settings
from .embeddings import embedding_service
from .vector_store import vector_store_service

# Load environment and disable tracing
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
    model="gemini-2.0-flash-exp",
    openai_client=external_provider
)


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
                f"[Source {idx}] (Relevance: {score:.2f})\n"
                f"Module: {payload['module']}\n"
                f"Section: {payload['section']}\n"
                f"Page: {payload['page']}\n"
                f"Content: {payload['text']}\n"
                f"---"
            )

        return "\n\n".join(context_parts)

    except Exception as e:
        return f"Error searching textbook: {str(e)}"


# Create RAG agent with Gemini
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
    model=gemini_model
)

# Test code commented out - cannot run at module level with uvicorn event loop
# run = Runner.run_sync(
#     rag_agent,
#     "What are the advantages of using NVIDIA Isaac Sim for robotics simulation compared to traditional simulators like Gazebo?"
# )

async def run_rag_agent(
    question: str,
    selected_text: Optional[str] = None,
    session_id: Optional[str] = None
) -> Dict:
    """
    Run the RAG agent to answer a question using textbook content

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
{selected_text[:2000]}  # Truncate to 2000 chars
---END SELECTED TEXT---

User's question about this selection: {question}

Please answer the question using both the selected text context and relevant information from the textbook."""
        else:
            enhanced_question = question

        # Run the agent
        result = await Runner.run(rag_agent, enhanced_question)

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

    except Exception as e:
        error_message = str(e)

        # Check for rate limit errors (429)
        if "429" in error_message or "quota" in error_message.lower() or "rate" in error_message.lower():
            return {
                "answer": "⚠️ API rate limit exceeded. The Gemini API has reached its quota limit. Please try again in a minute or contact the administrator to upgrade the API plan. For more info, visit: https://ai.google.dev/gemini-api/docs/rate-limits",
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
