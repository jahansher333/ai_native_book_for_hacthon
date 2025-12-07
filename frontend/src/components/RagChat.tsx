/**
 * RagChat Component
 * Simple chatbot widget for Physical AI textbook
 * Integrates with FastAPI backend RAG system
 */
import React, { useState, useEffect } from 'react';
import './RagChat.css';
import { API_ENDPOINTS } from '../config';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: string;
}

interface Citation {
  module: string;
  section: string;
  page: number;
  source_file: string;
  relevance_score: number;
}

const RagChat: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => {
    // Get or create session ID (only on client-side)
    if (typeof window === 'undefined') return ''; // SSR guard
    const stored = localStorage.getItem('rag_session_id');
    if (stored) return stored;
    const newId = crypto.randomUUID();
    localStorage.setItem('rag_session_id', newId);
    return newId;
  });

  // Capture text selection
  useEffect(() => {
    const handleSelection = () => {
      const selection = window.getSelection();
      const text = selection?.toString().trim() || '';
      if (text.length > 10 && text.length < 5000) {
        setSelectedText(text);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => document.removeEventListener('mouseup', handleSelection);
  }, []);

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage: Message = {
      role: 'user',
      content: input,
      timestamp: new Date().toISOString()
    };

    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      const response = await fetch(API_ENDPOINTS.QUERY, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: input,
          mode: selectedText ? 'selected' : 'general',
          selected_text: selectedText || null,
          session_id: sessionId
        })
      });

      if (!response.ok) throw new Error('Query failed');

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        timestamp: data.timestamp
      };

      setMessages(prev => [...prev, assistantMessage]);
      setSelectedText(''); // Clear selected text after use

    } catch (error) {
      const errorMessage: Message = {
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.',
        timestamp: new Date().toISOString()
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Floating Button */}
      {!isOpen && (
        <button
          className="rag-chat-button"
          onClick={() => setIsOpen(true)}
          aria-label="Open chatbot"
        >
          💬
        </button>
      )}

      {/* Chat Panel */}
      {isOpen && (
        <div className="rag-chat-panel">
          <div className="rag-chat-header">
            <h3>📚 AI Textbook Assistant</h3>
            <button
              onClick={() => setIsOpen(false)}
              className="rag-chat-close"
              aria-label="Close chatbot"
            >
              ✕
            </button>
          </div>

          <div className="rag-chat-messages">
            {messages.length === 0 && (
              <div className="rag-chat-welcome">
                <p>👋 Hi! I can answer questions about the Physical AI textbook.</p>
                <p>Try asking:</p>
                <ul>
                  <li>"What is ROS 2?"</li>
                  <li>"How much does the Jetson Orin Nano cost?"</li>
                  <li>Or select text on the page and ask about it!</li>
                </ul>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={`rag-chat-message rag-chat-message-${msg.role}`}>
                <div className="rag-chat-message-content">
                  {msg.content}
                </div>
                {msg.citations && msg.citations.length > 0 && (
                  <div className="rag-chat-citations">
                    <strong>Sources:</strong>
                    {msg.citations.map((cite, cidx) => (
                      <div key={cidx} className="rag-chat-citation">
                        📖 {cite.module} - {cite.section}, Page {cite.page}
                      </div>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className="rag-chat-message rag-chat-message-assistant">
                <div className="rag-chat-loading">
                  <span>Thinking</span>
                  <span className="rag-chat-dots">
                    <span>.</span><span>.</span><span>.</span>
                  </span>
                </div>
              </div>
            )}
          </div>

          <div className="rag-chat-input-container">
            {selectedText && (
              <div className="rag-chat-selected-text">
                <small>📌 Selected: {selectedText.slice(0, 50)}...</small>
                <button
                  onClick={() => setSelectedText('')}
                  className="rag-chat-clear-selection"
                >
                  ✕
                </button>
              </div>
            )}

            <div className="rag-chat-input-wrapper">
              <textarea
                className="rag-chat-input"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask a question about the textbook..."
                rows={2}
                disabled={isLoading}
              />
              <button
                onClick={sendMessage}
                disabled={!input.trim() || isLoading}
                className="rag-chat-send"
                aria-label="Send message"
              >
                ➤
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default RagChat;
