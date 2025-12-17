from fastapi import APIRouter, HTTPException, BackgroundTasks
import time
import uuid
from ...models.query import QueryRequest, QueryResponse
from ...services.query_processor import QueryProcessor

router = APIRouter()

# Initialize the query processor
query_processor = QueryProcessor()


@router.post("/query", response_model=QueryResponse)
async def process_query(
    query_request: QueryRequest,
    background_tasks: BackgroundTasks
):
    """
    Process a user query through the RAG pipeline and return structured context.
    """
    try:
        # Generate a unique query ID
        query_id = str(uuid.uuid4())

        # Record start time for performance monitoring
        start_time = time.time()

        # Validate and process the query through the RAG pipeline (task T014 & T020)
        result = await query_processor.validate_and_process(query_request)

        # Calculate processing time
        processing_time = time.time() - start_time

        # Create and return the response
        response = QueryResponse(
            context=result["context"],
            retrieved_chunks=result["retrieved_chunks"],
            session_id=query_request.session_id or str(uuid.uuid4()),
            processing_time=processing_time,
            query_id=query_id
        )

        return response

    except ValueError as e:
        # Handle validation errors (task T021)
        raise HTTPException(status_code=422, detail=str(e))
    except Exception as e:
        # Handle general errors (task T021)
        raise HTTPException(status_code=500, detail=f"Internal server error: {str(e)}")


@router.get("/health")
async def health_check():
    """
    Health check endpoint to verify the service is running.
    """
    return {"status": "healthy", "service": "query-processing"}