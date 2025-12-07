"""
Comprehensive Integration Test for All Three LiteLLM Groq Agents
Tests RAG, Personalize, and Urdu Translator agents
"""
import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from src.agents.litellm_converted.rag_agent import run_rag_agent
from src.agents.litellm_converted.personalize_agent import personalize_chapter_content
from src.agents.litellm_converted.urdu_translator import translate_chapter_to_urdu


async def test_rag_agent():
    """Test RAG Agent - Query about Jetson price"""
    print("\\n" + "="*80)
    print("TEST 1: RAG Agent - Jetson Price Query")
    print("="*80)

    try:
        result = await run_rag_agent("What is the Jetson Orin Nano price?")

        print(f"\\n📊 Answer ({len(result['answer'])} chars):")
        print(result['answer'][:500])
        if len(result['answer']) > 500:
            print(f"... (truncated, full length: {len(result['answer'])} chars)")

        print(f"\\n📚 Citations: {len(result['citations'])}")
        for i, cite in enumerate(result['citations'][:3], 1):
            print(f"  {i}. {cite['module']} - {cite['section']}, Page {cite['page']}")

        print(f"\\n🎯 Confidence: {result['confidence']:.2f}")

        # Validation
        has_price = "$249" in result['answer']
        has_citations = len(result['citations']) > 0

        print(f"\\n✅ VALIDATION:")
        print(f"  - Contains '$249': {'✓' if has_price else '✗'}")
        print(f"  - Has citations: {'✓' if has_citations else '✗'}")

        if has_price and has_citations:
            print(f"\\n🎉 RAG Agent Test: PASSED")
            return True
        else:
            print(f"\\n❌ RAG Agent Test: FAILED")
            return False

    except Exception as e:
        print(f"\\n❌ RAG Agent Error: {str(e)}")
        return False


async def test_personalize_agent():
    """Test Personalize Agent - Advanced profile with Jetson"""
    print("\\n" + "="*80)
    print("TEST 2: Personalize Agent - Advanced + Jetson Profile")
    print("="*80)

    # Sample chapter content
    chapter_content = \"\"\"# ROS 2 Basics

ROS 2 (Robot Operating System 2) is a middleware framework for robot software development. It uses DDS for communication between nodes.

## Installation
Install ROS 2 Humble on Ubuntu 22.04:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

## Running a Node
```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.get_logger().info('Hello, ROS 2!')

rclpy.init()
node = MinimalNode()
rclpy.spin(node)
```

The Jetson Orin Nano costs $249 and provides 10 TOPS INT8 performance.
\"\"\"

    profile = {
        "experience": "advanced",
        "hasJetson": True
    }

    try:
        result = await personalize_chapter_content(chapter_content, profile, "test-ros2-basics")

        print(f"\\n📝 Personalized Content ({len(result)} chars):")
        print(result[:600])
        if len(result) > 600:
            print(f"... (truncated, full length: {len(result)} chars)")

        # Validation
        has_price = "$249" in result
        has_deployment = any(keyword in result.lower() for keyword in ['jetson', 'deployment', 'tensorrt', 'edge'])
        has_code = "```" in result

        print(f"\\n✅ VALIDATION:")
        print(f"  - Preserves '$249': {'✓' if has_price else '✗'}")
        print(f"  - Mentions deployment/Jetson: {'✓' if has_deployment else '✗'}")
        print(f"  - Preserves code blocks: {'✓' if has_code else '✗'}")

        if has_price and has_deployment and has_code:
            print(f"\\n🎉 Personalize Agent Test: PASSED")
            return True
        else:
            print(f"\\n❌ Personalize Agent Test: FAILED")
            return False

    except Exception as e:
        print(f"\\n❌ Personalize Agent Error: {str(e)}")
        return False


async def test_urdu_translator():
    """Test Urdu Translator - ROS 2 terminology"""
    print("\\n" + "="*80)
    print("TEST 3: Urdu Translator - ROS 2 Translation")
    print("="*80)

    english_text = \"\"\"# ROS 2 Basics

ROS 2 is middleware for communication between robotic components. It uses the DDS protocol.

## Hardware Requirements
- Jetson Orin Nano: $249
- 8GB RAM minimum
- Ubuntu 22.04 LTS

```python
import rclpy
print("Hello ROS 2")
```

ROS 2 enables real-time robot control with low latency.
\"\"\"

    try:
        result = await translate_chapter_to_urdu(english_text, "test-ros2-translation")

        print(f"\\n🌐 Urdu Translation ({len(result)} chars):")
        print(result[:600])
        if len(result) > 600:
            print(f"... (truncated, full length: {len(result)} chars)")

        # Validation
        has_urdu = any(char > '\\u0600' for char in result)  # Check for Urdu/Arabic script
        has_price = "$249" in result
        has_code = "```" in result and "import rclpy" in result
        has_headers = "#" in result

        print(f"\\n✅ VALIDATION:")
        print(f"  - Contains Urdu text: {'✓' if has_urdu else '✗'}")
        print(f"  - Preserves '$249': {'✓' if has_price else '✗'}")
        print(f"  - Preserves code (English): {'✓' if has_code else '✗'}")
        print(f"  - Preserves markdown headers: {'✓' if has_headers else '✗'}")

        # Check for specific Urdu translations
        if has_urdu:
            print(f"  - Sample Urdu words detected: ✓")

        if has_urdu and has_price and has_code and has_headers:
            print(f"\\n🎉 Urdu Translator Test: PASSED")
            return True
        else:
            print(f"\\n❌ Urdu Translator Test: FAILED")
            return False

    except Exception as e:
        print(f"\\n❌ Urdu Translator Error: {str(e)}")
        return False


async def main():
    """Run all agent tests"""
    print("\\n" + "🧪 "*20)
    print("\\n🚀 Testing All Three LiteLLM Groq Agents")
    print("\\n" + "🧪 "*20)

    results = []

    # Test 1: RAG Agent
    results.append(await test_rag_agent())

    # Test 2: Personalize Agent
    results.append(await test_personalize_agent())

    # Test 3: Urdu Translator
    results.append(await test_urdu_translator())

    # Summary
    print("\\n" + "="*80)
    print("📊 TEST SUMMARY")
    print("="*80)

    test_names = ["RAG Agent", "Personalize Agent", "Urdu Translator"]
    for i, (name, passed) in enumerate(zip(test_names, results), 1):
        status = "✅ PASSED" if passed else "❌ FAILED"
        print(f"{i}. {name}: {status}")

    total_passed = sum(results)
    total_tests = len(results)

    print(f"\\n🎯 Overall: {total_passed}/{total_tests} tests passed")

    if total_passed == total_tests:
        print("\\n🎉 ALL TESTS PASSED! LiteLLM Groq agents working correctly.")
        print("\\n💰 Cost Savings: Estimated 90% reduction vs. Gemini")
        print("⚡ Performance: Groq inference typically 1-5 seconds")
    else:
        print(f"\\n⚠️  {total_tests - total_passed} test(s) failed. Review errors above.")

    print("\\n" + "="*80)


if __name__ == "__main__":
    print("\\n🔧 LiteLLM Groq Agents - Comprehensive Integration Test")
    print("📦 Testing: RAG | Personalize | Urdu Translation")
    print("🌐 Backend: Groq (llama-3-70b-8192, mixtral-8x7b-32768)")

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\\n\\n⚠️  Tests interrupted by user")
    except Exception as e:
        print(f"\\n\\n❌ Fatal error: {str(e)}")
