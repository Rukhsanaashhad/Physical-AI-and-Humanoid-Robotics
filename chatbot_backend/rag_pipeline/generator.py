# # chatbot_backend/rag_pipeline/generator.py
# # GEMINI VERSION — 100% FREE + FAST + WORKING

# import os
# import re
# from typing import List, Dict, Any
# from dotenv import load_dotenv
# import google.generativeai as genai


# load_dotenv()

# # Gemini setup – bas GOOGLE_API_KEY chahiye (jo tune daal di hai)
# genai.configure(api_key=os.getenv("GOOGLE_API_KEY"))

# class Generator:
#     def __init__(self):
#         self.model = genai.GenerativeModel('gemini-1.5-flash')

#     def _extract_citations(self, text: str) -> List[str]:
#         citation_pattern = re.compile(r'\(Source: (.*?)\)')
#         return citation_pattern.findall(text)

#     def generate_response(self, query: str, context_chunks: List[str]) -> Dict[str, Any]:
#         if not context_chunks:
#             return {"response": "I cannot answer this question based on the available book content.", "citations": []}

#         context = "\n".join(context_chunks)
        
#         prompt = f"""You are a helpful assistant that answers questions based ONLY on the following context from a book. 
# If the answer is not in the context, say you don't know.
# Always add citations at the end in this format: (Source: Chapter X, Section Y)

# Context:
# {context}

# Question: {query}

# Answer:"""

#         try:
#             response = self.model.generate_content(
#                 prompt,
#                 generation_config={
#                     "temperature": 0.3,
#                     "max_output_tokens": 800,
#                 }
#             )
            
#             generated_text = response.text.strip()
#             citations = self._extract_citations(generated_text)
            
#             # Citations hata ke clean response bhej do
#             clean_response = re.sub(r'\(Source: .*?\)', '', generated_text).strip()

#             return {
#                 "response": clean_response or generated_text,
#                 "citations": citations
#             }
            
#         except Exception as e:
#             print(f"Gemini error: {e}")
#             return {"response": "Sorry, main abhi jawab nahi de pa raha. Thodi der baad try karo.", "citations": []}


# # Test karne ke liye (optional)
# if __name__ == "__main__":
#     gen = Generator()
#     result = gen.generate_response("What is URDF?", ["URDF is XML format for robot description in ROS."])
#     print(result)