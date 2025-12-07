# # chatbot_backend/rag_pipeline/text_extractor.py
# import os
# import frontmatter
# from markdown import markdown
# from bs4 import BeautifulSoup

# def extract_text_from_markdown(file_path: str):
#     """
#     Extracts text content and front matter from a Markdown or MDX file.
#     Converts Markdown content to plain text.
#     """
#     with open(file_path, 'r', encoding='utf-8') as f:
#         post = frontmatter.load(f)

#     # Convert markdown content to HTML, then to plain text to remove formatting
#     html = markdown(post.content)
#     soup = BeautifulSoup(html, 'html.parser')
#     plain_text = soup.get_text()

#     return {
#         "text_content": plain_text.strip(),
#         "front_matter": post.metadata,
#         "file_path": file_path
#     }

# def extract_all_book_content(docs_dir: str = '../../docs'): # Relative path to Docusaurus docs
#     """
#     Extracts text content from all Markdown and MDX files in the specified directory.
#     """
#     all_documents = []
#     for root, _, files in os.walk(docs_dir):
#         for file_name in files:
#             if file_name.endswith(('.md', '.mdx')):
#                 file_path = os.path.join(root, file_name)
#                 document_data = extract_text_from_markdown(file_path)
#                 all_documents.append(document_data)
#     return all_documents

# if __name__ == "__main__":
#     # Example usage:
#     # This assumes you run this script from the chatbot_backend/ directory
#     # and the Docusaurus docs are in the project root's docs/ directory.
#     docs_path = os.path.join(os.path.dirname(__file__), '../../docs')
#     extracted_docs = extract_all_book_content(docs_path)
#     for doc in extracted_docs:
#         print(f"--- Extracted from: {doc['file_path']} ---")
#         print(f"Title: {doc['front_matter'].get('title', 'N/A')}")
#         print(f"Content length: {len(doc['text_content'])} characters")
#         # print(doc['text_content'][:200]) # Print first 200 chars
#         print("\n")
