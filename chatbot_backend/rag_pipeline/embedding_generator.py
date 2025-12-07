# chatbot_backend/rag_pipeline/embedding_generator.py
import os
from typing import List, Dict
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

class EmbeddingGenerator:
    def __init__(self):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = "text-embedding-ada-002" # Or a newer, more capable model

    def generate_embedding(self, text: str) -> List[float]:
        """Generates an embedding for a given text using OpenAI API."""
        if not text.strip():
            return [] # Return empty list for empty text

        response = self.client.embeddings.create(
            input=[text],
            model=self.model
        )
        return response.data[0].embedding

    def generate_embeddings_for_chunks(self, chunked_documents: List[Dict]) -> List[Dict]:
        """
        Generates embeddings for a list of chunked documents.
        Adds a 'vector_embedding' attribute to each chunk.
        """
        for doc_chunk in chunked_documents:
            text_to_embed = doc_chunk.get("text_content", "")
            embedding = self.generate_embedding(text_to_embed)
            doc_chunk["vector_embedding"] = embedding
        return chunked_documents

if __name__ == "__main__":
    # Example usage:
    # Ensure OPENAI_API_KEY is set in your .env file
    generator = EmbeddingGenerator()

    sample_chunks = [
        {
            "chunk_id": "doc1-chunk-0",
            "text_content": "ROS 2 is a flexible framework for writing robot software.",
            "front_matter": {"title": "ROS Overview", "slug": "/module-1/ros2-overview"},
            "file_path": "docs/module-1/ros2-overview.md",
            "chunk_number": 0
        },
        {
            "chunk_id": "doc1-chunk-1",
            "text_content": "It's a collection of tools, libraries, and conventions.",
            "front_matter": {"title": "ROS Overview", "slug": "/module-1/ros2-overview"},
            "file_path": "docs/module-1/ros2-overview.md",
            "chunk_number": 1
        }
    ]

    chunks_with_embeddings = generator.generate_embeddings_for_chunks(sample_chunks)
    for chunk in chunks_with_embeddings:
        print(f"Chunk ID: {chunk['chunk_id']}")
        print(f"Embedding size: {len(chunk['vector_embedding'])}")
        print(f"Embedding (first 5 values): {chunk['vector_embedding'][:5]}...\n")
