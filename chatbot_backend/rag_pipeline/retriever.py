# chatbot_backend/rag_pipeline/retriever.py
from typing import List, Dict, Any
from .embedding_generator import EmbeddingGenerator
from chatbot_backend.qdrant_client import QdrantManager # Changed to absolute import

class Retriever:
    def __init__(self, collection_name: str = "book_chunks"):
        self.embedding_generator = EmbeddingGenerator()
        self.qdrant_manager = QdrantManager(collection_name=collection_name)

    def retrieve_relevant_chunks(self, query_text: str, limit: int = 5, filters: Dict[str, Any] = None) -> List[Dict[str, Any]]:
        """
        Generates an embedding for the query and retrieves relevant chunks from Qdrant.
        """
        query_embedding = self.embedding_generator.generate_embedding(query_text)
        if not query_embedding:
            return []

        search_results = self.qdrant_manager.search_vectors(query_embedding, limit=limit, filters=filters)
        
        # Extract payload content for RAG
        relevant_chunks = []
        for hit in search_results:
            payload = hit['payload']
            relevant_chunks.append({
                "text_content": payload.get("text_content"),
                "chapter": payload.get("chapter"),
                "section": payload.get("section"),
                "title": payload.get("title"),
                "slug": payload.get("slug"),
                "score": hit['score']
            })
        return relevant_chunks

if __name__ == "__main__":
    # Example usage:
    # Ensure OPENAI_API_KEY, QDRANT_URL, QDRANT_API_KEY are set in your .env file
    retriever = Retriever()

    # Before running this, ensure data has been ingested using data_ingestor.py
    # manager = QdrantManager()
    # if not manager.client.collection_exists(collection_name="book_chunks"):
    #     print("Collection 'book_chunks' does not exist. Please run data_ingestor.py first.")
    # else:
    #     query = "What is ROS 2 architecture?"
    #     results = retriever.retrieve_relevant_chunks(query, limit=3)
    #     print(f"\nRelevant chunks for query: '{query}'")
    #     for i, chunk in enumerate(results):
    #         print(f"--- Result {i+1} (Score: {chunk['score']:.2f}) ---")
    #         print(f"Title: {chunk['title']}")
    #         print(f"Chapter/Section: {chunk['chapter']}/{chunk['section']}")
    #         print(f"Content: {chunk['text_content'][:200]}...")
    #         print("-" * 20)
