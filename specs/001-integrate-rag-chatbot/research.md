# Research Findings: RAG Chatbot Integration

## 1. Text Extraction and Chunking

**Decision**: N/A (Research in Progress)
**Rationale**: N/A
**Alternatives Considered**: N/A
**Task**: Investigate best practices and libraries for programmatically extracting clean text from Markdown files and strategies for optimal chunking (e.g., fixed-size, semantic, recursive) to maintain context for vector embedding.

## 2. Vector Embedding Generation

**Decision**: N/A (Research in Progress)
**Rationale**: N/A
**Alternatives Considered**: N/A
**Task**: Research and compare various open-source or commercial text embedding models (e.g., OpenAI Embeddings, Sentence-BERT, Cohere Embeddings). Evaluate performance, cost, and suitability for embedding book content, ensuring good semantic similarity capture for retrieval.

## 3. Qdrant Integration

**Decision**: N/A (Research in Progress)
**Rationale**: N/A
**Alternatives Considered**: N/A
**Task**: Investigate best practices for managing Qdrant collections (e.g., indexing strategies, payload storage), performing efficient semantic search queries, and handling filtering for specific citations within a FastAPI application.

## 4. FastAPI Best Practices for RAG

**Decision**: N/A (Research in Progress)
**Rationale**: N/A
**Alternatives Considered**: N/A
**Task**: Research best practices for structuring a FastAPI application for RAG, including dependency injection for Qdrant/Neon clients, asynchronous API design, error handling, and secure API key management.

## 5. Docusaurus Chat Widget Integration

**Decision**: N/A (Research in Progress)
**Rationale**: N/A
**Alternatives Considered**: N/A
**Task**: Investigate techniques for building an interactive chat widget in React/TypeScript that communicates with a FastAPI backend. Focus on UI/UX considerations, state management for conversations, and mechanisms for displaying citations and handling selected text queries within Docusaurus.
