import os
from dotenv import load_dotenv
# Load env vars first
load_dotenv()

from fastapi import FastAPI, HTTPException, status
from openai import AsyncOpenAI, APIConnectionError, RateLimitError
from pydantic import BaseModel
from qdrant_client import QdrantClient, models
from qdrant_client.http.exceptions import UnexpectedResponse

app = FastAPI()

# 1. Setup OpenAI (Gemini) Client
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
if not GEMINI_API_KEY:
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail="GEMINI_API_KEY is not set in environment variables."
    )

openai_client = AsyncOpenAI(
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/",
    api_key=GEMINI_API_KEY,
)

# 2. Setup Qdrant Client
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if not QDRANT_URL:
    print("❌ Warning: QDRANT_URL not found. Ensure .env is loaded.")

qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

COLLECTION_NAME = "my_documents"

class ChatRequest(BaseModel):
    message: str

async def get_embeddings(text: str):
    try:
        response = await openai_client.embeddings.create(
            model="text-embedding-004",
            input=text,
            dimensions=768 # Force 768 to match Qdrant config
        )
        return response.data[0].embedding
    except Exception as e:
        print(f"Error generating embedding: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Embedding error: {str(e)}"
        )

@app.get("/")
async def read_root():
    return {"message": "RAG Backend Server is running!"}

@app.post("/chat")
async def chat(request: ChatRequest):
    print(f"📩 Query: {request.message}")
    
    # 1. Generate Embedding
    query_embedding = await get_embeddings(request.message)

    try:
        # 2. Search Qdrant
        search_result = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_embedding,
            limit=3
        )
        
        # Extract text from the payload
        context_docs = [hit.payload["text"] for hit in search_result if hit.payload]
        
        if not context_docs:
            print("⚠️ No relevant context found in Qdrant.")
            context_text = "No specific context found in the textbook."
        else:
            context_text = "\n\n".join(context_docs)

    except Exception as e:
        print(f"🔥 Qdrant Error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Database error: {str(e)}"
        )

    # 3. Generate Answer
    try:
        response = await openai_client.chat.completions.create(
            model="gemini-2.5-flash", # Use the fast, free model
            messages=[
                {
                    "role": "system", 
                    "content": "You are a helpful expert teacher for a Robotics course. Answer the user's question clearly based ONLY on the provided context below. If the answer isn't in the context, say so."
                },
                {
                    "role": "user", 
                    "content": f"Context:\n{context_text}\n\nQuestion: {request.message}"
                }
            ]
        )
        
        answer = response.choices[0].message.content
        return {"response": answer}

    except Exception as e:
        print(f"🔥 Gemini Error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"LLM generation error: {str(e)}"
        )