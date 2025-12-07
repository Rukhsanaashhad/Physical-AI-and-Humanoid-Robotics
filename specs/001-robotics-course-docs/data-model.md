# Data Model: Enhance Robotics Book Docs

## Entities

### Course Content
**Description**: The primary educational material for the "Physical AI and Humanoid Robotics" course.
**Attributes**:
-   `Title`: String, main heading of the content.
-   `FilePath`: String, path to the Markdown file.
-   `Content`: Markdown text, the actual educational material.
-   `Module`: String, the module it belongs to (e.g., "ROS 2").
-   `Submodule`: String, the submodule it belongs to (e.g., "Installation").
-   `Keywords`: Array of Strings, for search and categorization.
-   `ROS2Examples`: Array of references to `ROS 2 Code Examples`.
-   `InteractiveComponents`: Array of references to `Interactive Components`.
**Relationships**:
-   Contains `ROS 2 Code Examples`.
-   Contains `Interactive Components`.
-   Belongs to a hierarchical structure (Modules, Submodules).

### Documentation Platform
**Description**: The system responsible for rendering, hosting, and presenting the `Course Content`. This is primarily Docusaurus.
**Attributes**:
-   `Technology`: String, "Docusaurus".
-   `IntegrationFramework`: String, "SpaceKit".
-   `Configuration`: JSON/YAML structure for Docusaurus settings (e.g., sidebar, plugins, themes).
-   `StaticAssets`: Array of FilePaths, references to images, videos, 3D models.
**Relationships**:
-   Renders `Course Content`.
-   Hosts `ROS 2 Code Examples` and `Interactive Components`.

### ROS 2 Code Examples
**Description**: Snippets or full code examples demonstrating ROS 2 concepts and applications.
**Attributes**:
-   `CodeSnippet`: String, the actual code.
-   `Language`: String, e.g., "Python", "C++".
-   `FilePath`: String, optional, if the code is in a separate file.
-   `Context`: String, description of what the example demonstrates.
-   `Version`: String, ROS 2 distribution version compatibility.
-   `Runnable`: Boolean, indicates if the example is designed for interactive execution.
**Relationships**:
-   Embedded within `Course Content`.

### Interactive Components
**Description**: Elements designed to enhance user engagement and learning, such as quizzes, embedded simulations, or interactive diagrams.
**Attributes**:
-   `Type`: String, e.g., "Quiz", "3DSimulation", "InteractiveDiagram".
-   `Content`: JSON/HTML/JS, the data or code for the interactive element.
-   `EmbeddedUrl`: String, optional, if the component is hosted externally.
-   `Context`: String, description of the learning objective.
-   `Dependencies`: Array of Strings, required libraries or frameworks for the component.
**Relationships**:
-   Embedded within `Course Content`.
