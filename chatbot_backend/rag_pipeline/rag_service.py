# from typing import List, Dict, Any

# # Assuming these modules are in the same rag_pipeline directory
# from .retriever import Retriever
# from .generator import Generator
# from neon_db_client import NeonDBManager # Changed to absolute import

# class RAGService:
#     def __init__(self, collection_name: str = "book_chunks"):
#         self.retriever = Retriever(collection_name=collection_name)
#         self.generator = Generator()
#         self.db_manager = NeonDBManager() # Initialize DB Manager
#         self.conversation_id = None # To store current conversation ID, or retrieve from session
#                                     # For simplicity, will create new or use a static one for now.

#     def chat_with_rag(self, query: str, selected_text: str = None) -> Dict[str, Any]:
#         """
#         Combines retrieval and generation to answer a user query.
#         If selected_text is provided, uses it as the primary context.
#         Also logs the conversation to Neon DB.
#         """
#         # Ensure a conversation ID exists
#         if not self.conversation_id:
#             # In a real app, user_id would be passed from frontend
#             self.conversation_id = self.db_manager.create_conversation(user_id="anonymous") 

#         context_chunks_content: List[str] = []
#         citations: List[Dict[str, str]] = []

#         # Log user query *before* retrieval/generation attempt for traceability
#         self.db_manager.add_message(
#             conversation_id=self.conversation_id,
#             sender="user",
#             message_text=query,
#             selected_text=selected_text,
#             raw_query=query
#         )

#         if selected_text:
#             # If selected text is provided, use it as the main context
#             context_chunks_content.append(selected_text)
#             # For selected text, we can't easily get precise citations without more sophisticated parsing
#             # For now, just indicate it was user-selected
#             citations.append({"chapter": "User Selected Text", "section": "N/A"})
#             print(f"Using selected text as primary context: {selected_text[:100]}...")
#         else:
#             # Retrieve relevant chunks from the knowledge base
#             retrieved_data = self.retriever.retrieve_relevant_chunks(query, limit=5)
            
#             if not retrieved_data:
#                 # Log bot response for no relevant data
#                 self.db_manager.add_message(
#                     conversation_id=self.conversation_id,
#                     sender="bot",
#                     message_text="I cannot find relevant information in the book to answer your question.",
#                     citation="N/A"
#                 )
#                 return {
#                     "response": "I cannot find relevant information in the book to answer your question.",
#                     "citation": "N/A",
#                     "source_docs": []
#                 }

#             # Extract text content and prepare citations
#             for chunk in retrieved_data:
#                 context_chunks_content.append(chunk["text_content"])
#                 citations.append({
#                     "chapter": chunk.get("title", chunk.get("slug")),
#                     "section": chunk.get("section", "N/A"),
#                     "slug": chunk.get("slug", "N/A")
#                 })
#             print(f"Retrieved {len(context_chunks_content)} chunks.")

#         # Generate response using the retrieved context
#         response_data = self.generator.generate_response(query, context_chunks_content)
#         response_text = response_data["response"]
#         generated_citations = response_data["citations"]

#         # Combine citations from retriever and generator
#         all_citations = []
#         for c in citations:
#             if c["chapter"] == "User Selected Text":
#                 all_citations.append(c["chapter"])
#             else:
#                 all_citations.append(f"{c['chapter']} (Section: {c['section']})")
#         all_citations.extend(generated_citations) # Add citations from generator
        
#         # Deduplicate citations
#         unique_citations = list(set(all_citations))

#         # Log bot response
#         self.db_manager.add_message(
#             conversation_id=self.conversation_id,
#             sender="bot",
#             message_text=response_text,
#             citation="; ".join(unique_citations) if unique_citations else "N/A"
#         )

#         return {
#             "response": response_text,
#             "citation": "; ".join(unique_citations) if unique_citations else "N/A",
#             "source_docs": citations # Return raw citation data for potential frontend use
#         }

# if __name__ == "__main__":
#     # Example usage:
#     # Ensure all environment variables are set and Qdrant has ingested data
#     # (e.g., by running data_ingestor.py first)
#     rag_service = RAGService()

#     print("\n--- Testing general query ---")
#     general_query = "What are the benefits of ROS 2 over ROS 1?"
#     response = rag_service.chat_with_rag(general_query)
#     print(f"Query: {general_query}")
#     print(f"Response: {response['response']}")
#     print(f"Citation: {response['citation']}")

#     print("\n--- Testing selected text query ---")
#     selected_text_query = "Explain this concept in detail."
#     selected_text_context = "The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all aspects of a robot. It's crucial for robotic applications."
#     response_selected = rag_service.chat_with_rag(selected_text_query, selected_text=selected_text_context)
#     print(f"Query: {selected_text_query}")
#     print(f"Selected Text: {selected_text_context}")
#     print(f"Response: {response_selected['response']}")
#     print(f"Citation: {response_selected['citation']}")

#     print("\n--- Testing query with no relevant info ---")
#     no_info_query = "What is the capital of France?"
#     response_no_info = rag_service.chat_with_rag(no_info_query)
#     print(f"Query: {no_info_query}")
#     print(f"Response: {response_no_info['response']}")
#     print(f"Citation: {response_no_info['citation']}")
