# Tasks: RAG Chatbot Integration

**Input**: Design documents from `specs/001-integrate-rag-chatbot/`
**Prerequisites**: `plan.md` (required), `spec.md` (required for user stories), `data-model.md`, `contracts/`, `research.md`, `quickstart.md`

**Tests**: Test tasks are NOT explicitly requested in the feature specification for this iteration. Testing will be incorporated within implementation tasks.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [TaskID] [P?] [Story?] Description with file path`

-   **[P]**: Can run in parallel (different files, no dependencies)
-   **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
-   Include exact file paths in descriptions

---

## Phase 1: Setup (Project Initialization & Environment)

**Purpose**: Establish the core project environment and install initial dependencies.

-   [ ] T001 Create `requirements.txt` for Python dependencies in `chatbot-backend/requirements.txt`
-   [ ] T002 Install Python dependencies specified in `chatbot-backend/requirements.txt`
-   [ ] T003 Ensure `.env` file for backend credentials exists and is configured in `chatbot-backend/.env`
-   [ ] T004 Verify Docusaurus frontend setup and Node.js dependencies (already done)

---

## Phase 2: Foundational (Research & Core Infrastructure)

**Purpose**: Conduct initial research to inform implementation decisions and set up core infrastructure. These tasks are prerequisites for effective user story implementation.

-   [ ] T005 [P] Research Text Extraction and Chunking techniques and libraries (relevant to FR-001) in `specs/001-integrate-rag-chatbot/research.md`
-   [ ] T006 [P] Research Vector Embedding Generation models and strategies (relevant to FR-002) in `specs/001-integrate-rag-chatbot/research.md`
-   [ ] T007 [P] Research Qdrant integration best practices for collections and semantic search (relevant to FR-003, FR-004) in `specs/001-integrate-rag-chatbot/research.md`
-   [ ] T008 [P] Research FastAPI best practices for RAG architecture, DI, error handling (relevant to FR-005) in `specs/001-integrate-rag-chatbot/research.md`
-   [ ] T009 [P] Research Docusaurus Chat Widget integration techniques and UI/UX (relevant to US4, FR-011) in `specs/001-integrate-rag-chatbot/research.md`

---

## Phase 3: User Story 1 - Ask a Question to the Chatbot (P1) [US1]

**Goal**: The chatbot can answer general questions based on book content and provide citations.

**Independent Test**: A user can ask a question, and the chatbot provides a relevant answer with citations.

### Implementation for User Story 1

-   [ ] T010 [P] [US1] Implement text extraction from Docusaurus `docs/` folder in `chatbot-backend/rag_pipeline/text_extractor.py` (FR-001)
-   [ ] T011 [P] [US1] Implement chunking strategy for extracted text in `chatbot-backend/rag_pipeline/text_chunker.py` (FR-001)
-   [ ] T012 [P] [US1] Develop vector embedding generation for text chunks in `chatbot-backend/rag_pipeline/embedding_generator.py` (FR-002)
-   [ ] T013 [P] [US1] Implement Qdrant client for storing embeddings and chunked data in `chatbot-backend/qdrant_client.py` (FR-003)
-   [ ] T014 [US1] Create Qdrant collection and upload initial book embeddings in `chatbot-backend/rag_pipeline/data_ingestor.py`
-   [ ] T015 [US1] Implement semantic search logic using Qdrant client in `chatbot-backend/rag_pipeline/retriever.py` (FR-004)
-   [ ] T016 [US1] Integrate OpenAI GPT model for response generation in `chatbot-backend/rag_pipeline/generator.py` (FR-006)
-   [ ] T017 [US1] Implement core RAG pipeline combining retriever and generator in `chatbot-backend/rag_pipeline/rag_service.py`
-   [ ] T018 [US1] Develop FastAPI endpoint `/chat` to handle incoming queries and return RAG response in `chatbot-backend/main.py` (FR-005, contracts/chatbot_api.yaml)

---

## Phase 4: User Story 2 - Query Selected Text (P1) [US2]

**Goal**: The chatbot can answer questions based exclusively on user-selected text.

**Independent Test**: A user can select text, ask a question, and receive an answer constrained to the selected text.

### Implementation for User Story 2

-   [ ] T019 [US2] Modify `/chat` FastAPI endpoint to accept `selected_text` parameter in `chatbot-backend/main.py` (FR-010, contracts/chatbot_api.yaml)
-   [ ] T020 [US2] Update RAG pipeline to use `selected_text` as primary context for response generation in `chatbot-backend/rag_pipeline/rag_service.py` (FR-010)
-   [ ] T021 [US2] Implement frontend logic to capture selected text and send with query in `src/components/ChatWidget.tsx` (FR-010)

---

## Phase 5: User Story 4 - Seamless Integration into Docusaurus (P1) [US4]

**Goal**: The chatbot widget is fully integrated into the Docusaurus frontend as a floating, accessible component.

**Independent Test**: The chat widget is visible on documentation pages, non-obtrusive, and responsive to user interaction.

### Implementation for User Story 4

-   [ ] T022 [US4] Create Docusaurus React component for the Chat Widget UI in `src/components/ChatWidget.tsx` (FR-011)
-   [ ] T023 [US4] Implement communication from Chat Widget to FastAPI backend `/chat` endpoint in `src/components/ChatWidget.tsx`
-   [ ] T024 [US4] Integrate ChatWidget component into Docusaurus Layout (`src/theme/Layout/index.tsx`) as a floating widget (FR-011)

---

## Phase 6: User Story 3 - View Chat History (P2) [US3]

**Goal**: The chatbot can store and display a user's conversation history.

**Independent Test**: A user can interact with the chatbot, close it, reopen it, and see their previous conversation.

### Implementation for User Story 3

-   [ ] T025 [US3] Develop Neon PostgreSQL client for storing conversations and interactions in `chatbot-backend/neon_db_client.py` (FR-007, FR-008)
-   [ ] T026 [US3] Implement logging of user queries and chatbot responses to Neon DB in `chatbot-backend/rag_pipeline/rag_service.py` (FR-007)
-   [ ] T027 [US3] Create FastAPI endpoint to retrieve chat history from Neon DB in `chatbot-backend/main.py` (FR-008)
-   [ ] T028 [US3] Implement frontend logic to display chat history in `src/components/ChatWidget.tsx` (US3)

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Finalize features, enhance user experience, and prepare for deployment.

-   [ ] T029 Implement citation extraction and formatting within RAG response in `chatbot-backend/rag_pipeline/generator.py` (FR-009)
-   [ ] T030 Update frontend to display citations prominently in `src/components/ChatWidget.tsx` (FR-009)
-   [ ] T031 Implement analytics tracking of user queries in `chatbot-backend/neon_db_client.py`
-   [ ] T032 Enhance error handling for API failures and RAG pipeline issues in `chatbot-backend/main.py`
-   [ ] T033 Document deployment procedures for FastAPI backend (e.g., Docker, cloud) in `quickstart.md`
-   [ ] T034 Review all code for security best practices (API key handling, input validation).

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Phase 1 (Setup)**: No dependencies - can start immediately.
-   **Phase 2 (Foundational)**: Depends on Setup completion - BLOCKS all user story phases.
-   **Phase 3 (US1)**: Depends on Foundational completion - BLOCKS Phase 4 (US2), Phase 5 (US4), Phase 6 (US3) as it implements core RAG and the /chat endpoint.
-   **Phase 4 (US2)**: Depends on Phase 3 (US1) completion (modifies existing endpoint).
-   **Phase 5 (US4)**: Depends on Phase 3 (US1) completion (integrates with /chat endpoint).
-   **Phase 6 (US3)**: Depends on Phase 3 (US1) completion (uses RAG responses, requires database).
-   **Phase 7 (Final)**: Depends on all preceding user story phases being substantially complete.

### User Story Dependencies

-   **US1 (Ask a Question)**: No direct dependencies on other user stories.
-   **US2 (Query Selected Text)**: Depends on US1 (enhances /chat endpoint, requires RAG service).
-   **US3 (View Chat History)**: Depends on US1 (requires RAG responses for logging, database setup).
-   **US4 (Seamless Integration)**: Depends on US1 (integrates with US1's /chat endpoint).

---

## Parallel Opportunities

-   **Phase 2 Research Tasks**: T005, T006, T007, T008, T009 can be conducted in parallel.
-   **User Story 1 Internal Tasks**: T010, T011, T012, T013 (initial setup for RAG pipeline components) can be worked on concurrently, but the RAG pipeline construction and `/chat` endpoint development (T014-T018) are generally sequential.
-   **Backend vs. Frontend Development**: Once the core `/chat` endpoint is stable (end of US1), frontend development (US4, US3) can progress in parallel with further backend refinements (US2, US3).

---

## Implementation Strategy

### Incremental Delivery & Prioritization

1.  **Phase 1: Setup**: Establish environment.
2.  **Phase 2: Foundational**: Research and initial core infrastructure setup.
3.  **Phase 3: User Story 1 (P1 - Core RAG)**: Implement the basic RAG pipeline and `/chat` endpoint.
4.  **Phase 5: User Story 4 (P1 - Docusaurus Integration)**: Integrate the basic chat widget with the `/chat` endpoint.
5.  **Validation Point**: Basic chatbot interaction is functional in Docusaurus.
6.  **Phase 4: User Story 2 (P1 - Selected Text)**: Enhance the RAG pipeline and frontend to handle selected text.
7.  **Phase 6: User Story 3 (P2 - Chat History)**: Implement database logging and history display.
8.  **Phase 7: Final Phase**: Polish, citations, and error handling.

This strategy prioritizes core chatbot functionality and frontend integration for rapid value delivery, followed by enhancements and supporting features.
