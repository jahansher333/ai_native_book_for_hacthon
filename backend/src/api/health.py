"""
Health check API endpoint
Monitors status of all dependencies
"""
from fastapi import APIRouter
from ..models.queries import HealthResponse
from ..services.vector_store import vector_store_service
from ..services.session_manager import session_manager

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Check health of all services

    Returns:
        HealthResponse with overall status and individual service statuses
    """
    services = {}

    # Check Qdrant
    try:
        qdrant_healthy = vector_store_service.check_health()
        services["qdrant"] = "healthy" if qdrant_healthy else "unhealthy"
    except:
        services["qdrant"] = "unhealthy"

    # Check Neon Postgres
    try:
        neon_healthy = session_manager.check_health()
        services["neon"] = "healthy" if neon_healthy else "unhealthy"
    except:
        services["neon"] = "unhealthy"

    # Check Gemini API (assume healthy if configured)
    from ..config import settings
    services["gemini"] = "healthy" if settings.gemini_api_key else "unhealthy"

    # Determine overall status
    if all(status == "healthy" for status in services.values()):
        overall_status = "healthy"
    elif any(status == "unhealthy" for status in services.values()):
        overall_status = "degraded"
    else:
        overall_status = "unhealthy"

    return HealthResponse(
        status=overall_status,
        services=services
    )
