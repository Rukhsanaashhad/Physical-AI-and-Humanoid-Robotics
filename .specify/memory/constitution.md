<!--
Sync Impact Report (2025-12-05)
Version change: 1.1.0 -> 1.2.0
Modified principles:
  - PROJECT_NAME (Updated from "RAG Chatbot for Physical AI Book" to "Physical AI Book with Chatbot Integration")
  - Required Technology Stack (Refined component breakdown)
Added sections: None
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md (⚠ pending)
  - .specify/templates/spec-template.md (⚠ pending)
  - .specify/templates/tasks-template.md (⚠ pending)
  - .specify/templates/commands/*.md (⚠ pending)
Follow-up TODOs:
  - TODO(SECURITY_PRIVACY): Define specific security and privacy compliance standards relevant to user data and LLM interactions.
  - TODO(SCALABILITY): Define specific performance SLAs and scaling targets.
-->
# Physical AI Book with Chatbot Integration Constitution

## Core Principles

### I. Modularity and Component-Based Architecture
The RAG chatbot system MUST be designed as a collection of modular, independently deployable, and testable components. Communication between components MUST be clearly defined via APIs or messaging. This facilitates independent development, scaling, and maintenance of individual system parts.

### II. Data-Driven Development and Traceability
All critical data, including conversations, chat history, user queries, analytics, and document metadata, MUST be stored in persistent and traceable databases (e.g., Neon PostgreSQL). Vector embeddings and chunked documents for RAG retrieval MUST be managed in a dedicated vector database (e.g., Qdrant Cloud). This ensures data integrity, auditability, and supports iterative development through data analysis.

### III. Content-Centric Accuracy (RAG)
The chatbot MUST answer questions ONLY from the provided book content. Responses MUST be directly verifiable against the source material and provide citations (chapter and section) to enable users to locate the source. The system MUST support querying based on selected text from the book, ensuring answers are strictly derived from the selected context.

### IV. Robust Frontend Integration
The chatbot MUST be seamlessly integrated into the Docusaurus frontend as a floating widget, providing an intuitive and accessible user experience within the book's interface. The integration MUST minimize impact on Docusaurus performance and maintain the book's existing styling and navigation.

### V. Security and Privacy
All system components and data handling procedures MUST adhere to best practices for security and privacy. User interactions and stored data MUST be protected against unauthorized access, modification, or disclosure.
**API Key Security Rules:**
1.  API Keys (e.g., QDRANT_API_KEY) MUST NEVER be printed to logs or console output.
2.  API Keys MUST NEVER be hardcoded directly into source code.
3.  API Keys MUST ONLY be stored in environment variables (e.g., `.env` files for local development, secure secrets management for deployment).
(TODO: Define specific security and privacy compliance standards relevant to user data and LLM interactions.)

### VI. Scalability
The architecture MUST support horizontal and vertical scalability for all components (FastAPI backend, Neon, Qdrant, OpenAI interactions) to handle increasing user load and data volumes. Performance bottlenecks MUST be identified and addressed proactively. (TODO: Define specific performance SLAs and scaling targets.)

## Required Technology Stack

The project components and their underlying technologies are:
-   **Docusaurus Book**: The primary content platform (already exists).
-   **FastAPI Backend (Python)**: The core backend service for the chatbot and RAG pipeline (needs to be created).
    -   Utilizes: OpenAI GPT model for generation, Embedding pipeline for book modules + chapters.
    -   Integrates with: Neon PostgreSQL, Qdrant Cloud.
-   **Chat Widget**: The frontend component for the chatbot (integrated inside the Docusaurus book).
-   **Neon PostgreSQL**: For structured data storage (conversations, chat history, analytics, user queries, document metadata).
    -   Endpoint: `ep-cool-tooth-a1r8tjlg`
    -   Branch: `br-wandering-glade-a17l70wo`
    -   Final DB URL: `postgresql://neondb_owner:<password>@ep-cool-tooth-a1r8tjlg.ap-south-1.aws.neon.tech/neondb?sslmode=require` (stored in `.env`)
-   **Qdrant Cloud**: For vector embeddings and semantic search.
    -   Stores: Vector embeddings for all book chapters, chunked documents for semantic search, similarity results for RAG retrieval.
    -   Environment Variables: `QDRANT_URL` and `QDRANT_API_KEY` (stored in `.env` files).
        -   `QDRANT_URL="https://c784f436-bcc2-470b-b265-5aa8559ff465.europe-west3-0.gcp.cloud.qdrant.io"`
        -   `QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.J_7bfTiG3Q3MXTqBGBxVdBFrA0btaV11W6NA2CHtlLk"` (Example, actual key stored in .env)

## Chatbot Functionality and Constraints

The RAG Chatbot MUST provide the following functionalities and adhere to these constraints:
-   Answer questions based ONLY on the book's content.
-   Support selected paragraph querying, providing answers exclusively from the selected text.
-   Search book content using semantic embeddings.
-   Provide citations (chapter and section) for all answers.
-   Log all user interactions into the Neon PostgreSQL database.
-   Operate as a floating widget integrated within the Docusaurus sidebar.

## Governance

Constitution supersedes all other practices; Amendments require documentation, approval, migration plan. All PRs/reviews MUST verify compliance; Complexity MUST be justified; Use documentation for runtime development guidance.

**Version**: 1.2.0 | **Ratified**: 2025-12-05 | **Last Amended**: 2025-12-05