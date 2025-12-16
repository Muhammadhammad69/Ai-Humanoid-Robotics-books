"""
Error handling and retry mechanisms for ChatKit integration
"""
import asyncio
import logging
from typing import Callable, Any, Optional
from functools import wraps


def retry_on_failure(max_retries: int = 3, delay: float = 1.0, backoff: float = 2.0):
    """
    Decorator for retrying functions that may fail
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        async def wrapper(*args, **kwargs) -> Any:
            retries = 0
            current_delay = delay

            while retries < max_retries:
                try:
                    return await func(*args, **kwargs)
                except Exception as e:
                    retries += 1
                    if retries >= max_retries:
                        logging.error(f"Function {func.__name__} failed after {max_retries} retries: {str(e)}")
                        raise e

                    logging.warning(f"Attempt {retries} failed: {str(e)}. Retrying in {current_delay}s...")
                    await asyncio.sleep(current_delay)
                    current_delay *= backoff  # Exponential backoff

            return None
        return wrapper
    return decorator


class ChatKitErrorHandler:
    """
    Error handler for ChatKit operations
    """

    @staticmethod
    async def handle_stream_error(error: Exception, context: Optional[dict] = None):
        """
        Handle errors during streaming operations
        """
        logging.error(f"Stream error occurred: {str(error)}", extra=context)
        # Implement specific error handling logic based on error type
        if "timeout" in str(error).lower():
            return {"error": "Request timeout", "retry": True}
        elif "connection" in str(error).lower():
            return {"error": "Connection error", "retry": True}
        else:
            return {"error": "Processing error", "retry": False}

    @staticmethod
    async def handle_attachment_error(error: Exception, attachment_id: str):
        """
        Handle errors related to attachment processing
        """
        logging.error(f"Attachment processing error for ID {attachment_id}: {str(error)}")
        return {"error": "Attachment processing failed", "attachment_id": attachment_id}

    @staticmethod
    def log_error(error: Exception, operation: str = "unknown", context: Optional[dict] = None):
        """
        Generic error logging
        """
        error_context = {
            "operation": operation,
            "error_type": type(error).__name__,
            "error_message": str(error)
        }
        if context:
            error_context.update(context)

        logging.error(f"ChatKit operation failed: {operation}", extra=error_context)