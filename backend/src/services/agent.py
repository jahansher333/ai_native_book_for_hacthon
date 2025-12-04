from openai import OpenAI
from typing import Dict, Any, List
import json
import os
import time


class RAGAgent:
    """RAG Agent using OpenAI function calling for textbook Q&A"""

    def __init__(self, vector_store_service, embedding_service):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.vector_store = vector_store_service
        self.embedding_service = embedding_service
        self.model = "gpt-4o-mini"

        # Define search tool schema
        self.search_tool = {
            "type": "function",
            "function": {
                "name": "search_textbook",
                "description": "Search the Physical AI & Humanoid Robotics textbook for relevant content",
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "The search query or question"
                        },
                        "selected_text": {
                            "type": "string",
                            "description": "Additional context from user's text selection"
                        },
                        "module_filter": {
                            "type": "string",
                            "enum": ["01-ros2", "02-gazebo-unity", "03-isaac", "04-vla", "hardware", "all"],
                            "description": "Filter results by specific module"
                        }
                    },
                    "required": ["query"]
                }
            }
        }

    def search_textbook_function(self, query: str, selected_text: str = "", module_filter: str = "all") -> List[Dict[str, Any]]:
        """Function called by the agent to search textbook"""
        # Combine query with selected text if provided
        search_query = query
        if selected_text:
            search_query = f"{selected_text}\n\nQuestion: {query}"

        # Generate embedding
        embedding = self.embedding_service.embed_text(search_query)

        # Search Qdrant
        results = self.vector_store.search(
            query_embedding=embedding,
            limit=5,
            module_filter=module_filter
        )

        return results

    def run_rag_agent(
        self,
        question: str,
        selected_text: str = "",
        module_filter: str = "all"
    ) -> Dict[str, Any]:
        """
        Run RAG agent with function calling

        Args:
            question: User's question
            selected_text: Optional selected text from page
            module_filter: Module to search in

        Returns:
            Dict with answer, sources, and confidence
        """
        start_time = time.time()

        system_prompt = """You are a helpful assistant for the Physical AI & Humanoid Robotics textbook.

CRITICAL RULES:
1. Answer ONLY using the provided context from the search_textbook tool
2. Do NOT use external knowledge
3. Always cite sources using the format: [Source: Module X - Title]
4. If the answer is not in the context, say "I don't have information about that in the textbook."
5. For robot control questions, emphasize edge deployment (Jetson) for <10ms latency
6. Cite accurate hardware prices (e.g., "$249 Jetson Orin Nano")
"""

        user_message = question
        if selected_text:
            user_message = f"Selected text context:\n{selected_text}\n\nQuestion: {question}"

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message}
        ]

        # Initial call to agent
        response = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            tools=[self.search_tool],
            tool_choice="auto"
        )

        sources = []
        confidence = 0.5

        # Handle tool calls (agentic loop)
        while response.choices[0].finish_reason == "tool_calls":
            tool_call = response.choices[0].message.tool_calls[0]

            if tool_call.function.name == "search_textbook":
                args = json.loads(tool_call.function.arguments)
                search_results = self.search_textbook_function(
                    query=args.get("query", question),
                    selected_text=args.get("selected_text", selected_text),
                    module_filter=args.get("module_filter", module_filter)
                )

                # Store sources
                sources = [
                    {
                        "title": f"{result['module']} - {result['chapter']}",
                        "url": result["source_url"],
                        "chunk_id": result["chunk_id"],
                        "score": result["score"]
                    }
                    for result in search_results
                ]

                # Calculate confidence (average of top scores)
                if sources:
                    confidence = sum(s["score"] for s in sources) / len(sources)

                # Feed results back to agent
                messages.append(response.choices[0].message)
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": json.dumps(search_results)
                })

                response = self.client.chat.completions.create(
                    model=self.model,
                    messages=messages,
                    tools=[self.search_tool]
                )

        answer = response.choices[0].message.content
        processing_time_ms = int((time.time() - start_time) * 1000)

        return {
            "answer": answer,
            "sources": sources,
            "confidence": confidence,
            "processing_time_ms": processing_time_ms
        }
