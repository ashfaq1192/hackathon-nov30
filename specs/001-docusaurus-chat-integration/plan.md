# Implementation Plan: Docusaurus Chat Integration

**Branch**: `001-docusaurus-chat-integration` | **Date**: 2025-11-30 | **Spec**: specs/001-docusaurus-chat-integration/spec.md
**Input**: Feature specification from `specs/001-docusaurus-chat-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integrate a chat interface into the Docusaurus book, connecting to a local backend service to provide answers based on the book's content. This involves creating a chat component, configuring its connection, adding a floating chat button, and ensuring correct handling of backend responses.

## Technical Context

**Language/Version**: Frontend: React/TypeScript (Docusaurus 3.9); Backend: Python 3.12 (FastAPI).
**Primary Dependencies**: Frontend: React, Docusaurus, OpenAI ChatKit. Backend: FastAPI, OpenAI client, Qdrant-client.
**Storage**: Qdrant (Vector DB) for backend knowledge base.
**Testing**: Frontend: Jest/React Testing Library (assumed). Backend: pytest.
**Target Platform**: Frontend: Web browser. Backend: Linux server.
**Project Type**: Web application (Frontend + Backend).
**Performance Goals**: FCP < 1.5s, LCP < 2.5s for Docusaurus site. Chat response within 5 seconds.
**Constraints**: Backend on `http://localhost:8000/chat`. Docusaurus environment allows custom components.
**Scale/Scope**: Chat integration for the "Physical AI & Humanoid Robotics Textbook" content.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

-   **I. Spec-Driven First**: PASS - The plan is derived directly from the feature specification.
-   **II. Educational Tone**: N/A - This principle applies to content, not the technical implementation plan itself.
-   **III. Matrix-Style Skills**: PASS - The chat interface enables interaction with the RAG backend, which utilizes AI skills for content retrieval and generation.
-   **Technical Architecture**: PASS - The plan aligns with the defined technical architecture in the Constitution: Docusaurus (React/TypeScript) for the book content, OpenAI ChatKit for the chat UI, FastAPI (Python 3.12) for the Agent Backend, and Qdrant for the Knowledge Base.
-   **Authentication**: N/A - Authentication is not required for frontend chat interactions, as clarified by the user.

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-chat-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book-app/ # Docusaurus frontend
├── src/
│   ├── components/
│   │   └── ChatWidget/ # New React component for the chat interface
│   └── theme/ # Potentially for injecting the chat button
└── docusaurus.config.js # For configuration if needed

rag-backend/ # Existing FastAPI backend
├── main.py
├── ingest.py
├── requirements.txt
└── .venv/
```

**Structure Decision**: The chat interface will be implemented as a new React component `ChatWidget` within the existing `book-app/src/components/` directory. Configuration or injection points within Docusaurus (e.g., `docusaurus.config.js` or theme files) will be identified as part of implementation. The existing `rag-backend/` structure will be utilized.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
|           |            |                                     |

## Assumptions
- The backend service (`http://localhost:8000/chat`) is assumed to be fully functional and available.
- The Docusaurus environment is configured to allow the integration of custom components.
- The existing `rag-backend` provides `text-embedding-004` and `gemini-2.5-flash` models.
- The Qdrant collection "my_documents" is populated with the book content.
