---
id: 012
title: Amended constitution for RAG chatbot project
stage: constitution
date_iso: 2025-12-05
model: gemini-1.5-pro
feature: null
branch: null
user: null
command: |-
  /sp.constitution """
  PHASE 2 — RAG CHATBOT DEVELOPMENT FOR PHYSICAL AI BOOK

  ## 🎯 Project Goal
  Build and integrate a full Retrieval-Augmented Generation (RAG) chatbot into the published Docusaurus book.
  The chatbot must answer questions ONLY from the book’s content and support selected-text querying.

  ## 🗂 Required System Components
  1. Neon PostgreSQL (SQL database)
  2. Qdrant Cloud (Vector embeddings database)
  3. FastAPI backend (Python)
  4. OpenAI GPT model for generation
  5. Docusaurus frontend integration
  6. Embedding pipeline for book modules + chapters

  ## 🔐 Neon Setup (Completed)
  - Endpoint: ep-cool-tooth-a1r8tjlg
  - Branch: br-wandering-glade-a17l70wo
  - Final DB URL (stored in .env):
    postgresql://neondb_owner:<password>@ep-cool-tooth-a1r8tjlg.ap-south-1.aws.neon.tech/neondb?sslmode=require

  Neon will store:
  - conversations
  - chat history
  - analytics
  - user queries
  - document metadata

  ## 🧠 Qdrant (Next Component)
  Qdrant will store:
  - vector embeddings for all book chapters
  - chunked documents for semantic search
  - similarity results for RAG retrieval

  ## 🧩 What the Chatbot Must Do
  - Answer questions based ONLY on the book
  - Support selected paragraph → answer only from that text
  - Search book content using semantic embeddings
  - Provide citations (chapter and section)
  - Log user interactions into Neon
  - Work inside the Docusaurus sidebar as a floating widget

  ## 📌 Next Step
  Proceed to: “Qdrant Cloud Setup”
  """
labels:
  - constitution
  - project-setup
  - rag-chatbot
links: null
files_yaml:
  - .specify/memory/constitution.md
tests_yaml: null
prompt_text: |-
  ---
  description: Create or update the project constitution from interactive or provided principle inputs, ensuring all dependent templates stay in sync
  ---

  ## User Input

  ```text
  $ARGUMENTS
  ```

  You **MUST** consider the user input before proceeding (if not empty).

  ## Outline

  You are updating the project constitution at `.specify/memory/constitution.md`. This file is a TEMPLATE containing placeholder tokens in square brackets (e.g. `[PROJECT_NAME]`, `[PRINCIPLE_1_NAME]`). Your job is to (a) collect/derive concrete values, (b) fill the template precisely, and (c) propagate any amendments across dependent artifacts.

  Follow this execution flow:

  1. Load the existing constitution template at `.specify/memory/constitution.md`.
     - Identify every placeholder token of the form `[ALL_CAPS_IDENTIFIER]`.
     **IMPORTANT**: The user might require less or more principles than the ones used in the template. If a number is specified, respect that - follow the general template. You will update the doc accordingly.

  2. Collect/derive values for placeholders:
     - If user input (conversation) supplies a value, use it.
     - Otherwise infer from existing repo context (README, docs, prior constitution versions if embedded).
     - For governance dates: `RATIFICATION_DATE` is the original adoption date (if unknown ask or mark TODO), `LAST_AMENDED_DATE` is today if changes are made, otherwise keep previous.
     - `CONSTITUTION_VERSION` must increment according to semantic versioning rules:
       - MAJOR: Backward incompatible governance/principle removals or redefinitions.
       - MINOR: New principle/section added or materially expanded guidance.
       - PATCH: Clarifications, wording, typo fixes, non-semantic refinements.
     - If version bump type ambiguous, propose reasoning before finalizing.

  3. Draft the updated constitution content:
     - Replace every placeholder with concrete text (no bracketed tokens left except intentionally retained template slots that the project has chosen not to define yet—explicitly justify any left).
     - Preserve heading hierarchy and comments can be removed once replaced unless they still add clarifying guidance.
     - Ensure each Principle section: succinct name line, paragraph (or bullet list) capturing non‑negotiable rules, explicit rationale if not obvious.
     - Ensure Governance section lists amendment procedure, versioning policy, and compliance review expectations.

  4. Consistency propagation checklist (convert prior checklist into active validations):
     - Read `.specify/templates/plan-template.md` and ensure any "Constitution Check" or rules align with updated principles.
     - Read `.specify/templates/spec-template.md` for scope/requirements alignment—update if constitution adds/removes mandatory sections or constraints.
     - Read `.specify/templates/tasks-template.md` and ensure task categorization reflects new or removed principle-driven task types (e.g., observability, versioning, testing discipline).
     - Read each command file in `.specify/templates/commands/*.md` (including this one) to verify no outdated references (agent-specific names like CLAUDE only) remain when generic guidance is required.
     - Read any runtime guidance docs (e.g., `README.md`, `docs/quickstart.md`, or agent-specific guidance files if present). Update references to principles changed.

  5. Produce a Sync Impact Report (prepend as an HTML comment at top of the constitution file after update):
     - Version change: old → new
     - List of modified principles (old title → new title if renamed)
     - Added sections
     - Removed sections
     - Templates requiring updates (✅ updated / ⚠ pending) with file paths
     - Follow-up TODOs if any placeholders intentionally deferred.

  6. Validation before final output:
     - No remaining unexplained bracket tokens.
     - Version line matches report.
     - Dates ISO format YYYY-MM-DD.
     - Principles are declarative, testable, and free of vague language ("should" → replace with MUST/SHOULD rationale where appropriate).

  7. Write the completed constitution back to `.specify/memory/constitution.md` (overwrite).

  8. Output a final summary to the user with:
     - New version and bump rationale.
     - Any files flagged for manual follow-up.
     - Suggested commit message (e.g., `docs: amend constitution to vX.Y.Z (principle additions + governance update)`).

  Formatting & Style Requirements:

  - Use Markdown headings exactly as in the template (do not demote/promote levels).
  - Wrap long rationale lines to keep readability (<100 chars ideally) but do not hard enforce with awkward breaks.
  - Keep a single blank line between sections.
  - Avoid trailing whitespace.

  If the user supplies partial updates (e.g., only one principle revision), still perform validation and version decision steps.

  If critical info missing (e.g., ratification date truly unknown), insert `TODO(<FIELD_NAME>): explanation` and include in the Sync Impact Report under deferred items.

  Do not create a new template; always operate on the existing `.specify/memory/constitution.md` file.

  ---

  As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

  1) Determine Stage
     - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

  2) Generate Title and Determine Routing:
     - Generate Title: 3–7 words (slug for filename)
     - Route is automatically determined by stage:
       - `constitution` → `history/prompts/constitution/`
       - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
       - `general` → `history/prompts/general/`

  3) Create and Fill PHR (Shell first; fallback agent‑native)
     - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
     - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT.
     - If the script fails:
       - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
       - Allocate an ID; compute the output path based on stage from step 2; write the file
       - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

  4) Validate + report
     - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
     - On failure: warn, don't block. Skip only for `/sp.phr`.
response_text: |-
  The constitution file `.specify/memory/constitution.md` has been successfully updated with the RAG chatbot project details and prepended with a Sync Impact Report.

  -   **New version**: `1.0.0`
  -   **Version bump rationale**: Major version increment (1.0.0) as the constitution is being populated with concrete project-specific details for the first time, including new principles, required technology stack, and chatbot functionalities, which represent a significant redefinition of the project's foundational guidelines.
  -   **Files flagged for manual follow-up**:
      - `.specify/templates/plan-template.md` (⚠ pending)
      - `.specify/templates/spec-template.md` (⚠ pending)
      - `.specify/templates/tasks-template.md` (⚠ pending)
      - `.specify/templates/commands/*.md` (⚠ pending)
  -   **Suggested commit message**: `docs: amend constitution to v1.0.0 (initial RAG chatbot project details)`
outcome: The project constitution has been successfully updated to reflect the details of the RAG chatbot project, including its core principles, technology stack, and functionalities. A Sync Impact Report has been generated and flagged templates for manual follow-up.
evaluation: The constitution update was performed as requested, incorporating all provided details and addressing the versioning and impact reporting requirements.
---