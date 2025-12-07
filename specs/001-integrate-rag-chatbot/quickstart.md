# Quickstart Guide: RAG Chatbot Integration

This quickstart guide provides instructions for setting up and running the RAG Chatbot backend and integrating it with the Docusaurus frontend.

## Prerequisites

*   Python 3.10+
*   Node.js (LTS version)
*   npm (Node Package Manager)
*   Access to Neon PostgreSQL database (credentials in `.env`)
*   Access to Qdrant Cloud instance (credentials in `.env`)
*   OpenAI API Key (stored in `.env`)

## 1. Backend Setup (FastAPI)

Navigate to the `chatbot-backend/` directory:

```bash
cd chatbot-backend/
```

### 1.1 Install Python Dependencies

Install the required Python packages:

```bash
pip install fastapi uvicorn "uvicorn[standard]" python-dotenv openai qdrant-client psycopg2-binary
```
*(Note: `psycopg2-binary` is for PostgreSQL, `openai` for OpenAI API, `qdrant-client` for Qdrant, `python-dotenv` for `.env` management)*

### 1.2 Environment Variables

Create a `.env` file in the `chatbot-backend/` directory with your credentials:

```ini
# .env file in chatbot-backend/
NEON_DB_URL="postgresql://neondb_owner:<password>@ep-cool-tooth-a1r8tjlg.ap-south-1.aws.neon.tech/neondb?sslmode=require"
QDRANT_URL="https://c784f436-bcc2-470b-b265-5aa8559ff465.europe-west3-0.gcp.cloud.qdrant.io"
QDRANT_API_KEY="<your_qdrant_api_key>"
OPENAI_API_KEY="<your_openai_api_key>"
```
Replace `<password>`, `<your_qdrant_api_key>`, and `<your_openai_api_key>` with your actual credentials.

### 1.3 Run the Backend

Start the FastAPI server:

```bash
uvicorn main:app --reload
```

The backend API will be running at `http://localhost:8000`.

## 2. Frontend Setup (Docusaurus)

Navigate back to the project root directory:

```bash
cd ..
```

### 2.1 Install Node.js Dependencies

If you haven't already, install the Docusaurus dependencies:

```bash
npm install
```

### 2.2 Run the Frontend

Start the Docusaurus development server:

```bash
npm start
```

The Docusaurus site will be available at `http://localhost:3000` (or another port).

## 3. Chatbot Integration

The chat button is already integrated into the Docusaurus frontend and will appear on non-homepage documentation pages.
When you click the chat button, it will attempt to communicate with the FastAPI backend running at `http://localhost:8000/chat`.
(Note: The current implementation only shows a placeholder alert. Full integration with the backend will require further development.)

## 4. Deployment

This section will document the procedures for deploying the FastAPI backend and Docusaurus frontend to a production environment.

**TODO**: Add detailed deployment instructions (e.g., Dockerization, cloud provider specific steps for AWS/GCP/Azure, CI/CD pipeline integration).
