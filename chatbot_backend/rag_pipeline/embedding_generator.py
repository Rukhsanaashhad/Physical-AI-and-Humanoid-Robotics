# # chatbot_backend/rag_pipeline/embedding_generator.py
# # GEMINI EMBEDDING VERSION — FULLY WORKING + FREE

# import os
# from typing import List, Dict
# from dotenv import load_dotenv
# import google.generativeai as genai

# load_dotenv()

# # Google API key se setup
# genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

# class EmbeddingGenerator:
#     def __init__(self):
#         self.model = "models/embedding-001"  # Gemini ka official embedding model

#     def generate_embedding(self, text: str) -> List[float]:
#         """Single text ke liye embedding return karta hai"""
#         if not text.strip():
#             return [0.0] * 768  # fallback

#         try:
#             result = genai.embed_content(
#                 model=self.model,
#                 content=text,
#                 task_type="retrieval_document"
#             )
#             return result['embedding']
#         except Exception as e:
#             print(f"Embedding error: {e}")
#             return [0.0] * 768

#     def generate_embeddings_for_chunks(self, chunked_documents: List[Dict]) -> List[Dict]:
#         """Pura chunk list leke har ek ko embedding daal deta hai"""
#         for doc_chunk in chunked_documents:
#             text_to_embed = doc_chunk.get("text_content", "")
#             embedding = self.generate_embedding(text_to_embed)
#             doc_chunk["vector_embedding"] = embedding
#         return chunked_documents


# # Test karne ke liye (optional)
# if __name__ == "__main__":
#     gen = EmbeddingGenerator()
#     test_chunks = [
#         {"text_content": "ROS 2 ek robot software framework hai."},
#         {"text_content": "URDF robot ka description format hai."}
#     ]
#     result = gen.generate_embeddings_for_chunks(test_chunks)
#     print(f"Embedding size: {len(result[0]['vector_embedding'])}")
#     print("First 5 values:", result[0]['vector_embedding'][:5])