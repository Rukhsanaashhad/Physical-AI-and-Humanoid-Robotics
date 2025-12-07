# chatbot_backend/rag_pipeline/generator.py
import os
import re # Import regex module
from typing import List, Dict, Any # Import Any for potential payload details
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv() # Load environment variables from .env file

class Generator:
    def __init__(self):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = "gpt-3.5-turbo" # Or a newer, more capable model

    def _extract_citations(self, text: str) -> List[str]:
        """Extracts citations from the generated text based on a pattern."""
        # This regex looks for "Source: Chapter X, Section Y" or similar
        # Pattern needs to be robust for various citation formats OpenAI might generate
        citation_pattern = re.compile(r'\(Source: (.*?)\)')
        return citation_pattern.findall(text)

    def generate_response(self, query: str, context_chunks: List[str]) -> Dict[str, Any]:
        """
        Generates a natural language response using OpenAI's GPT model
        based on the user's query and provided context.
        Returns response text and extracted citations.
        """
        if not context_chunks:
            return {"response": "I cannot answer this question based on the available book content.", "citations": []}

        # Combine query and context for the prompt
        context = "\n".join(context_chunks)
        
        # Craft a prompt that encourages answering only from context and provides citations
        prompt_messages = [
            {"role": "system", "content": "You are a helpful assistant that answers questions based ONLY on the provided context from a book. If the answer is not in the context, state that you cannot answer from the provided information. Always provide citations at the end of your answer in the format '(Source: Chapter Title, Section Name)'."},
            {"role": "user", "content": f"Context from book:\n{context}\n\nUser Query: {query}\n\nAnswer:"}
        ]

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=prompt_messages,
                temperature=0.7, # Adjust creativity
                max_tokens=500 # Limit response length
            )
            generated_text = response.choices[0].message.content.strip()
            citations = self._extract_citations(generated_text)
            
            # Remove citations from the main response text for cleaner presentation
            response_text_without_citations = re.sub(r'\(Source: (.*?)\)', '', generated_text).strip()

            return {"response": response_text_without_citations, "citations": citations}
        except Exception as e:
            print(f"Error generating response from OpenAI: {e}")
            return {"response": "I apologize, but I encountered an error while trying to generate a response.", "citations": []}

if __name__ == "__main__":
    # Example usage:
    # Ensure OPENAI_API_KEY is set in your .env file
    generator = Generator()

    sample_query = "What is the purpose of URDF?"
    sample_context = [
        "The Unified Robot Description Format (URDF) is an XML format used in ROS to describe all aspects of a robot. It's crucial for robotic applications, enabling visualization, simulation, and planning algorithms to understand the robot's physical properties, kinematics, and dynamics.",
        "For humanoid robots, URDF becomes even more critical due to their complex articulated structures and need for precise modeling. (Source: Module 1, Chapter: URDF for Humanoids)"
    ]

    response_data = generator.generate_response(sample_query, sample_context)
    print(f"Query: {sample_query}")
    print(f"Response: {response_data['response']}")
    print(f"Citations: {response_data['citations']}")
