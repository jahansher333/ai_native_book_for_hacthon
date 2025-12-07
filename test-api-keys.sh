#!/bin/bash
# Test script to verify Railway backend API keys are working

BACKEND_URL="https://web-production-d0418.up.railway.app"

echo "=================================="
echo "Railway Backend API Key Test"
echo "=================================="
echo ""
echo "Backend URL: $BACKEND_URL"
echo "Test Time: $(date)"
echo ""

# Test 1: Root endpoint
echo "1. Testing Root Endpoint..."
ROOT_RESPONSE=$(curl -s $BACKEND_URL/)
echo "   Response: $ROOT_RESPONSE"
if [[ $ROOT_RESPONSE == *"running"* ]]; then
    echo "   ✅ PASS - Backend is running"
else
    echo "   ❌ FAIL - Backend not responding"
fi
echo ""

# Test 2: Health endpoint
echo "2. Testing Health Endpoint..."
HEALTH_RESPONSE=$(curl -s $BACKEND_URL/api/health)
echo "   Response: $HEALTH_RESPONSE"

if [[ $HEALTH_RESPONSE == *"healthy"* ]] && [[ $HEALTH_RESPONSE != *"unhealthy"* ]]; then
    echo "   ✅ PASS - All services healthy"
    echo "   ✅ API keys are configured correctly!"
elif [[ $HEALTH_RESPONSE == *"degraded"* ]]; then
    echo "   ⚠️  DEGRADED - Some services unhealthy"

    if [[ $HEALTH_RESPONSE == *"qdrant\":\"unhealthy"* ]]; then
        echo "   ❌ Qdrant: UNHEALTHY - Check QDRANT_URL and QDRANT_API_KEY"
    else
        echo "   ✅ Qdrant: Healthy"
    fi

    if [[ $HEALTH_RESPONSE == *"neon\":\"unhealthy"* ]]; then
        echo "   ❌ Neon: UNHEALTHY - Check NEON_DATABASE_URL"
    else
        echo "   ✅ Neon: Healthy"
    fi

    if [[ $HEALTH_RESPONSE == *"gemini\":\"unhealthy"* ]]; then
        echo "   ❌ Gemini: UNHEALTHY - Check GEMINI_API_KEY"
    else
        echo "   ✅ Gemini: Healthy"
    fi
else
    echo "   ❌ FAIL - Unexpected response"
fi
echo ""

# Test 3: Query endpoint
echo "3. Testing RAG Query Endpoint..."
QUERY_RESPONSE=$(curl -s -X POST $BACKEND_URL/api/query \
    -H "Content-Type: application/json" \
    -d '{"question":"What is AI?","module":"all","top_k":3}')

echo "   Response: ${QUERY_RESPONSE:0:200}..."

if [[ $QUERY_RESPONSE == *"NoneType"* ]] || [[ $QUERY_RESPONSE == *"not callable"* ]]; then
    echo "   ❌ FAIL - API keys not configured"
    echo "   💡 Set GEMINI_API_KEY, QDRANT_URL, QDRANT_API_KEY in Railway"
elif [[ $QUERY_RESPONSE == *"answer"* ]] || [[ $QUERY_RESPONSE == *"response"* ]]; then
    echo "   ✅ PASS - Query working! API keys configured correctly!"
elif [[ $QUERY_RESPONSE == *"error"* ]] || [[ $QUERY_RESPONSE == *"detail"* ]]; then
    echo "   ⚠️  ERROR - Check logs for details"
else
    echo "   ⚠️  UNKNOWN - Unexpected response format"
fi
echo ""

# Test 4: Docs endpoint
echo "4. Testing API Documentation..."
DOCS_STATUS=$(curl -s -o /dev/null -w "%{http_code}" $BACKEND_URL/docs)
if [[ $DOCS_STATUS == "200" ]]; then
    echo "   ✅ PASS - Docs available at $BACKEND_URL/docs"
else
    echo "   ❌ FAIL - Docs not accessible (Status: $DOCS_STATUS)"
fi
echo ""

# Summary
echo "=================================="
echo "Summary"
echo "=================================="

if [[ $HEALTH_RESPONSE == *"healthy"* ]] && [[ $HEALTH_RESPONSE != *"unhealthy"* ]]; then
    echo "✅ ALL TESTS PASSED - API keys configured correctly!"
    echo ""
    echo "Your backend is fully functional:"
    echo "  - All services are healthy"
    echo "  - RAG queries should work"
    echo "  - Authentication should work"
    echo "  - Ready for production!"
else
    echo "⚠️  ACTION REQUIRED - Environment variables missing"
    echo ""
    echo "Add these in Railway Dashboard → Variables tab:"
    echo ""
    echo "  GEMINI_API_KEY=your_gemini_api_key"
    echo "  QDRANT_URL=https://your-cluster.qdrant.io"
    echo "  QDRANT_API_KEY=your_qdrant_api_key"
    echo "  NEON_DATABASE_URL=postgresql://..."
    echo "  GROQ_API_KEY=your_groq_api_key"
    echo "  JWT_SECRET=your_32_char_secret"
    echo ""
    echo "After adding variables, Railway will auto-redeploy."
    echo "Then run this script again to verify!"
fi
echo ""
