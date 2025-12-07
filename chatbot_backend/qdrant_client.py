# chatbot_backend/qdrant_client.py
import os
from typing import List, Dict, Any
from qdrant_client import QdrantClient, models
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

class QdrantManager:
    def __init__(self, collection_name: str = "book_chunks"):
        self.collection_name = collection_name
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")

        if not self.qdrant_url or not self.api_key:
            raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in the .env file.")

        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.api_key,
        )

    def create_collection(self, vector_size: int = 1536): # Default for OpenAI's text-embedding-ada-002
        """
        Creates a Qdrant collection for storing embeddings.
        If the collection already exists, it does nothing.
        """
        try:
            self.client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=vector_size, distance=models.Distance.COSINE),
            )
            print(f"Collection '{self.collection_name}' created successfully.")
        except Exception as e:
            print(f"Error creating collection (might already exist): {e}")

    def upsert_vectors(self, documents: List[Dict]):
        """
        Upserts (inserts or updates) vectors along with their payloads into the Qdrant collection.
        Each document dict should contain 'chunk_id', 'vector_embedding', and metadata in 'front_matter'.
        """
        points = []
        for doc in documents:
            if not doc.get("vector_embedding"):
                print(f"Warning: Document {doc.get('chunk_id', 'Unknown')} has no embedding. Skipping.")
                continue

            payload = {
                "text_content": doc["text_content"],
                "file_path": doc["file_path"],
                "chunk_number": doc["chunk_number"],
                "title": doc["front_matter"].get("title"),
                "chapter": doc["front_matter"].get("chapter"), # Assuming chapter/section could be in front matter
                "section": doc["front_matter"].get("section"),
                "slug": doc["front_matter"].get("slug")
            }
            # Remove None values from payload
            payload = {k: v for k, v in payload.items() if v is not None}

            points.append(
                models.PointStruct(
                    id=doc["chunk_id"],
                    vector=doc["vector_embedding"],
                    payload=payload
                )
            )

        if points:
            operation_info = self.client.upsert(
                collection_name=self.collection_name,
                wait=True,
                points=points,
            )
            print(f"Upserted {len(points)} points to collection '{self.collection_name}': {operation_info}")
        else:
            print("No points to upsert.")

    def search_vectors(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """
        Searches the Qdrant collection for similar vectors.
        Returns a list of payloads of the most similar documents.
        """
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit,
            with_payload=True,
            with_vectors=False # No need to retrieve vectors in search results
        )
        results = []
        for hit in search_result:
            results.append({
                "score": hit.score,
                "payload": hit.payload
            })
        return results

if __name__ == "__main__":
    # Example usage:
    # Ensure QDRANT_URL and QDRANT_API_KEY are set in your .env file
    # This example requires a dummy embedding for testing
    manager = QdrantManager()
    manager.create_collection()

    # Dummy documents with embeddings
    dummy_documents = [
        {
            "chunk_id": "test_doc_1",
            "text_content": "This is a test document about ROS 2 features.",
            "front_matter": {"title": "Test Doc 1", "chapter": "Test Chapter", "section": "Test Section"},
            "file_path": "test/path/doc1.md",
            "chunk_number": 0,
            "vector_embedding": [0.1] * 1536 # Replace with actual embedding
        },
        {
            "chunk_id": "test_doc_2",
            "text_content": "Another document on FastAPI and Python.",
            "front_matter": {"title": "Test Doc 2", "chapter": "Test Chapter", "section": "Another Section"},
            "file_path": "test/path/doc2.md",
            "chunk_number": 0,
            "vector_embedding": [0.2] * 1536 # Replace with actual embedding
        }
    ]
    # manager.upsert_vectors(dummy_documents)

    # Example search (requires a query embedding)
    # query_embedding = [0.15] * 1536 # Replace with actual query embedding
    # search_results = manager.search_vectors(query_embedding)
    # for res in search_results:
    #     print(f"Score: {res['score']}, Content: {res['payload'].get('text_content', '')[:100]}...")
