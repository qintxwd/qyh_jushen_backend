"""QYH Jushen Control Plane - lightweight health and readiness endpoints."""

from fastapi import APIRouter

from app.schemas.response import ApiResponse, ErrorCodes, error_response, success_response
from app.services.health_checker import ServiceStatus, get_health_checker

router = APIRouter(tags=["health"])


@router.get("/health", response_model=ApiResponse)
async def health_check() -> ApiResponse:
    """Liveness + dependency summary."""
    checker = get_health_checker()
    results = await checker.check_all()

    services = {}
    for name, result in results.items():
        services[name] = {
            "status": result.status.value,
            "latency_ms": result.latency_ms,
            "message": result.message,
        }

    overall_status = "healthy"
    if any(r.status == ServiceStatus.UNHEALTHY for r in results.values()):
        overall_status = "unhealthy"
    elif any(r.status in (ServiceStatus.DEGRADED, ServiceStatus.UNKNOWN) for r in results.values()):
        overall_status = "degraded"

    return success_response(
        data={
            "status": overall_status,
            "services": services,
        },
        message="Health check completed",
    )


@router.get("/ready", response_model=ApiResponse)
async def readiness_check() -> ApiResponse:
    """Readiness requires critical dependencies to be healthy."""
    checker = get_health_checker()
    results = await checker.check_all()

    critical_services = ["data_plane", "ros2"]
    not_ready = []

    for service_name in critical_services:
        result = results.get(service_name)
        if result is None or result.status != ServiceStatus.HEALTHY:
            not_ready.append(
                {
                    "name": service_name,
                    "status": result.status.value if result else ServiceStatus.UNKNOWN.value,
                    "message": result.message if result else "missing health result",
                }
            )

    if not_ready:
        return error_response(
            code=ErrorCodes.OPERATION_FAILED,
            message="Service not ready",
            data={
                "status": "not_ready",
                "critical_failures": not_ready,
            },
        )

    return success_response(
        data={
            "status": "ready",
            "critical_services": critical_services,
        },
        message="Service ready",
    )
