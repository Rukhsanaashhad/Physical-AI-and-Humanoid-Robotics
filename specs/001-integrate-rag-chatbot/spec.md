# Feature Specification: RAG Chatbot Integration

**Feature Branch**: `001-integrate-rag-chatbot`  
**Created**: 2025-12-06  
**Status**: Draft  
**Input**: User description: "Build complete RAG chatbot for Physical AI Book: Components needed: 1. Text extraction from docs/ folder 2. Vector storage in Qdrant Cloud 3. Semantic search implementation 4. FastAPI endpoints for chat 5. Docusaurus integration Use existing: - Neon Postgres (conversation history) - Qdrant Cloud (vector database) - FastAPI backend structure Make it WORKING, not demo."

## User Scenarios & Testing (mandatory)

### User Story 1 - Ask a Question to the Chatbot (Priority: P1)

**Description**: As a book reader, I want to ask questions about the book content in natural language and receive concise, accurate answers, so that I can quickly understand concepts or find specific information without manually searching.

**Why this priority**: This is the core value proposition of the RAG chatbot.

**Independent Test**: The user can ask any question related to the book content and receive a relevant answer.

**Acceptance Scenarios**:

1.  **Given** the chatbot widget is visible, **When** the user types a question related to the book content (e.g., "What is ROS 2?"), **Then** the chatbot MUST provide an accurate answer derived solely from the book.
2.  **Given** the chatbot provides an answer, **When** the user reviews the answer, **Then** the chatbot MUST include citations (chapter and section) linking back to the source text in the book.

### User Story 2 - Query Selected Text (Priority: P1)

**Description**: As a book reader, I want to select a specific paragraph or section of text in the book and ask the chatbot to provide an answer or explanation based ONLY on that selected text, so that I can get context-specific information or clarification.

**Why this priority**: This is a direct explicit requirement and a powerful way to leverage RAG.

**Independent Test**: The user can select a text snippet and query the chatbot, receiving an answer exclusively from that snippet.

**Acceptance Scenarios**:

1.  **Given** the user has selected a portion of text in the book, **When** the user activates the chatbot and submits a query (e.g., "Explain this."), **Then** the chatbot MUST respond using ONLY information contained within the selected text.
2.  **Given** the chatbot responds to a selected text query, **Then** the chatbot MUST provide citations to the selected text's chapter and section.

### User Story 3 - View Chat History (Priority: P2)

**Description**: As a book reader, I want to view my previous conversations with the chatbot, so that I can recall past answers or continue a previous line of inquiry.

**Why this priority**: Enhances user experience and continuity.

**Independent Test**: The user can open the chatbot and see a log of their past interactions.

**Acceptance Scenarios**:

1.  **Given** the user has previously interacted with the chatbot, **When** the user opens the chatbot widget, **Then** the chatbot MUST display the history of their conversation(s).

### User Story 4 - Seamless Integration into Docusaurus (Priority: P1)

**Description**: As a book reader, I want the chatbot to be easily accessible and visually integrated within the Docusaurus book interface, so that I can use it without leaving the reading experience.

**Why this priority**: Core integration requirement.

**Independent Test**: The chatbot widget is visible, floating, and does not obstruct content on standard page layouts.

**Acceptance Scenarios**:

1.  **Given** a Docusaurus book page is loaded, **When** the user navigates through the book, **Then** a floating chatbot widget MUST be consistently available (e.g., in the sidebar or corner of the screen).
2.  **Given** the chatbot widget is active, **Then** it MUST not interfere with the readability or interactivity of the underlying Docusaurus content.

### Edge Cases

-   What happens when a question is asked that is not covered by the book's content? (Expected: Chatbot indicates it cannot answer from available content.)
-   How does the system handle very long selected text queries? (Expected: Chatbot processes up to a reasonable token limit, informs user if truncated.)
-   What happens if the backend API is unavailable? (Expected: Chatbot indicates service unavailability gracefully.)

## Requirements (mandatory)

### Functional Requirements

-   **FR-001**: The system MUST extract text content from the Docusaurus `docs/` folder.
-   **FR-002**: The system MUST generate vector embeddings for chunked book content.
-   **FR-003**: The system MUST store vector embeddings and chunked document metadata in Qdrant Cloud.
-   **FR-004**: The system MUST implement semantic search capabilities using Qdrant Cloud to retrieve relevant book content.
-   **FR-005**: The system MUST provide FastAPI endpoints for receiving user chat queries and returning chatbot responses.
-   **FR-006**: The system MUST integrate OpenAI GPT model for generating natural language responses.
-   **FR-007**: The system MUST log all user interactions (queries, responses, timestamps) into Neon Postgres.
-   **FR-008**: The system MUST store chat history and conversations in Neon Postgres.
-   **FR-009**: The system MUST identify and provide citations (chapter and section) for all generated answers.
-   **FR-010**: The system MUST support user-selected text as context for queries, ensuring responses are constrained to that text.
-   **FR-011**: The system MUST integrate the chatbot as a floating widget within the Docusaurus frontend.

### Key Entities (include if feature involves data)

-   **Book Document**: Represents a chapter or section of the Physical AI Book.
    -   Attributes: `id`, `text_content`, `chapter`, `section`, `vector_embedding`.
-   **User Query**: Represents a question posed by the user to the chatbot.
    -   Attributes: `id`, `user_id`, `query_text`, `timestamp`, `selected_context_text` (optional).
-   **Chatbot Response**: Represents the answer generated by the chatbot.
    -   Attributes: `id`, `query_id`, `response_text`, `timestamp`, `citations` (list of `chapter`/`section`), `source_embeddings_ids`.
-   **Conversation**: Represents a series of interactions between a user and the chatbot.
    -   Attributes: `id`, `user_id`, `start_time`, `end_time`.

## Success Criteria (mandatory)

### Measurable Outcomes

-   **SC-001**: 90% of user questions related to book content receive a relevant and accurate answer within 5 seconds.
-   **SC-002**: 100% of chatbot responses include accurate citations (chapter and section) that link to the source text.
-   **SC-003**: Chatbot response accuracy to selected text queries is 95% (answers are derived ONLY from selected text).
-   **SC-004**: The chatbot widget is accessible and functional on all Docusaurus documentation pages without obstructing primary content.
-   **SC-005**: All user interactions are logged successfully into Neon Postgres, and chat history is retrievable.