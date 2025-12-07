# # chatbot_backend/rag_pipeline/data_ingestor.py
# import os
# from dotenv import load_dotenv
# from typing import List, Dict, Any

# # Assuming these modules are in the same rag_pipeline directory
# from .text_extractor import extract_all_book_content
# from .text_chunker import chunk_documents
# from .embedding_generator import EmbeddingGenerator
# from qdrant_client import QdrantManager # Changed to absolute import

# load_dotenv() # Load environment variables from .env file

# def ingest_data_to_qdrant(docs_base_path: str = '../../docs'):
#     """
#     Orchestrates the data ingestion pipeline:
#     1. Extracts text from Docusaurus Markdown files.
#     2. Chunks the text.
#     3. Generates embeddings for chunks.
#     4. Creates a Qdrant collection and uploads the data.
#     """
#     print(f"Starting data ingestion from {docs_base_path}...")

#     # 1. Extract text content
#     print("Extracting text from Docusaurus documentation...")
#     extracted_docs = extract_all_book_content(docs_base_path)
#     print(f"Extracted {len(extracted_docs)} documents.")

#     if not extracted_docs:
#         print("No documents found to ingest.")
#         return

#     # 2. Chunk the text
#     print("Chunking documents...")
#     chunked_docs = chunk_documents(extracted_docs)
#     print(f"Generated {len(chunked_docs)} chunks.")

#     if not chunked_docs:
#         print("No chunks generated from documents.")
#         return

#     # 3. Generate embeddings
#     print("Generating embeddings for chunks (this may take a while)...")
#     embedding_generator = EmbeddingGenerator()
#     chunks_with_embeddings = embedding_generator.generate_embeddings_for_chunks(chunked_docs)
#     print("Embeddings generated.")

#     # 4. Create Qdrant collection and upload data
#     print(f"Connecting to Qdrant and upserting {len(chunks_with_embeddings)} chunks...")
#     qdrant_manager = QdrantManager(collection_name="book_chunks")
    
#     # Ensure collection exists
#     # Using a dummy vector_size, will be updated by actual embedding size
#     # A safer approach is to get the first embedding size dynamically
#     if chunks_with_embeddings and chunks_with_embeddings[0].get("vector_embedding"):
#         vector_size = len(chunks_with_embeddings[0]["vector_embedding"])
#     else:
#         print("Warning: Could not determine vector size from first chunk. Using default 1536.")
#         vector_size = 1536 # Default for text-embedding-ada-002

#     qdrant_manager.create_collection(vector_size=vector_size)
#     qdrant_manager.upsert_vectors(chunks_with_embeddings)
#     print("Data ingestion to Qdrant complete.")

# if __name__ == "__main__":
#     # Ensure you are running this from the chatbot_backend/ directory
#     # or adjust docs_base_path accordingly
#     current_dir = os.path.dirname(__file__)
#     docs_path_relative_to_rag_pipeline = os.path.join(current_dir, '../../docs')
#     ingest_data_to_qdrant(docs_path_relative_to_rag_pipeline)
