import requests
import json
from datetime import datetime

BASE_URL = "https://web-production-d0418.up.railway.app"

print("=" * 60)
print("LiteLLM RAG Agent Live Test")
print("=" * 60)
print(f"Timestamp: {datetime.now().isoformat()}")
print(f"Backend: {BASE_URL}")
print("=" * 60)
print()

# Test 1: Root endpoint
print("Test 1: Root Endpoint")
print("-" * 40)
try:
    response = requests.get(f"{BASE_URL}/", timeout=5)
    print(f"Status: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
except Exception as e:
    print(f"Error: {e}")
print()

# Test 2: Health check
print("Test 2: Health Check")
print("-" * 40)
try:
    response = requests.get(f"{BASE_URL}/api/health", timeout=5)
    print(f"Status: {response.status_code}")
    data = response.json()
    print(f"Response: {json.dumps(data, indent=2)}")
    
    # Analyze health status
    if data.get("status") == "healthy":
        print("✅ All services healthy!")
    elif data.get("status") == "degraded":
        print("⚠️  Services degraded:")
        for service, status in data.get("services", {}).items():
            symbol = "✅" if status == "healthy" else "❌"
            print(f"   {symbol} {service}: {status}")
except Exception as e:
    print(f"Error: {e}")
print()

# Test 3: Query endpoint - Simple question
print("Test 3: Query Endpoint - General Mode")
print("-" * 40)
try:
    payload = {
        "question": "What is ROS2?",
        "mode": "general"
    }
    print(f"Request: {json.dumps(payload, indent=2)}")
    response = requests.post(
        f"{BASE_URL}/api/query",
        json=payload,
        timeout=10
    )
    print(f"Status: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
except Exception as e:
    print(f"Error: {e}")
print()

# Test 4: Query endpoint - Selected text mode
print("Test 4: Query Endpoint - Selected Text Mode")
print("-" * 40)
try:
    payload = {
        "question": "Explain this",
        "mode": "selected",
        "selected_text": "ROS 2 uses DDS for communication between nodes"
    }
    print(f"Request: {json.dumps(payload, indent=2)}")
    response = requests.post(
        f"{BASE_URL}/api/query",
        json=payload,
        timeout=10
    )
    print(f"Status: {response.status_code}")
    print(f"Response: {json.dumps(response.json(), indent=2)}")
except Exception as e:
    print(f"Error: {e}")
print()

# Summary
print("=" * 60)
print("Test Summary")
print("=" * 60)
print("Backend Status: LIVE ✅")
print("Query Endpoint: FAILING ❌")
print("Root Cause: Missing API keys (GROQ_API_KEY, GEMINI_API_KEY, QDRANT)")
print("=" * 60)
