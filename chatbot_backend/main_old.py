# # chatbot_backend/main.py
# from fastapi import FastAPI, HTTPException
# from fastapi.middleware.cors import CORSMiddleware
# import os
# from pydantic import BaseModel
# from dotenv import load_dotenv
# from typing import List, Dict, Any
# from datetime import datetime

# from rag_pipeline.rag_service import RAGService 

# load_dotenv() # Load environment variables

# app = FastAPI()

# # Configure CORS (Cross-Origin Resource Sharing)
# origins = [
#     "http://localhost",
#     "http://localhost:3000",  # Docusaurus default development port
# ]

# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=origins,
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )

# # Initialize RAG Service
# rag_service = RAGService()

# # Pydantic model for incoming chat requests
# class ChatRequest(BaseModel):
#     query: str
#     selected_text: str | None = None # Optional selected text
#     conversation_id: str | None = None # Optional: to continue a conversation

# # Pydantic model for outgoing chat responses
# class ChatResponse(BaseModel):
#     response: str
#     citation: str
#     conversation_id: str | None = None

# # Pydantic model for conversation history message
# class HistoryMessage(BaseModel):
#     sender: str
#     message_text: str
#     timestamp: datetime
#     citation: str | None = None
#     selected_text: str | None = None
#     raw_query: str | None = None

# # Pydantic model for conversation history
# class ConversationHistory(BaseModel):
#     history: List[HistoryMessage]

# @app.get("/")
# async def read_root():
#     return {"message": "FastAPI Chatbot Backend is running!"}

# @app.post("/chat", response_model=ChatResponse)
# async def chat_with_rag(request: ChatRequest):
#     try:
#         if request.conversation_id:
#             rag_service.conversation_id = request.conversation_id
        
#         rag_response = rag_service.chat_with_rag(request.query, request.selected_text)
        
#         conversation_id = rag_service.conversation_id

#         return ChatResponse(
#             response=rag_response["response"],
#             citation=rag_response["citation"],
#             conversation_id=conversation_id
#         )
#     except HTTPException as e: # Catch explicit HTTPExceptions
#         raise e
#     except Exception as e: # Catch other unexpected errors
#         print(f"ERROR: Chat endpoint failed: {e}")
#         raise HTTPException(status_code=500, detail="Internal server error in RAG pipeline.")

# @app.get("/history/{conversation_id}", response_model=ConversationHistory)
# async def get_chat_history(conversation_id: str):
#     try:
#         history_data = rag_service.db_manager.get_conversation_history(conversation_id)
#         history_messages = [HistoryMessage(**msg) for msg in history_data]
#         return ConversationHistory(history=history_messages)
#     except Exception as e:
#         print(f"ERROR: History endpoint failed for conversation_id {conversation_id}: {e}")
#         raise HTTPException(status_code=500, detail="Could not retrieve conversation history.")
