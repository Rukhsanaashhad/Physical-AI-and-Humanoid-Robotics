# Quickstart Guide: Enhance Robotics Book Docs

This quickstart guide will cover the basic steps to set up and run the enhanced robotics book documentation locally.

## Prerequisites

- Node.js (LTS version)
- npm or Yarn

## Setup

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```

2.  **Install dependencies:**
    ```bash
    npm install
    # or
    yarn install
    ```

## Running Locally

To start the local development server:

```bash
npm run start
# or
yarn start
```

This will open the documentation site in your browser at `http://localhost:3000` (or another port if 3000 is in use).

## Building for Deployment

To build a static version of the documentation for deployment:

```bash
npm run build
# or
yarn build
```

The static files will be generated in the `build/` directory. These files can then be deployed to any static hosting service (e.g., GitHub Pages, Netlify, Vercel).

## Adding/Updating Content

-   All course content is written in Markdown (`.md` or `.mdx`) files, located in the `docs/` directory.
-   To add a new page, create a new Markdown file in the appropriate directory within `docs/`.
-   Update `sidebars.js` to include the new page in the navigation.
-   Refer to the Docusaurus documentation for advanced features and customization.

## Integrating SpaceKit

-   SpaceKit components can be integrated directly into `.mdx` files. Refer to SpaceKit documentation for usage.
-   Ensure SpaceKit dependencies are correctly installed and configured in `docusaurus.config.js` if custom setup is required.

## Integrating ROS 2 Code Examples

-   ROS 2 code examples can be embedded as code blocks using standard Markdown.
-   For interactive examples, custom React components may be developed and integrated into `.mdx` files.

## Testing

-   Unit tests for custom React components are located in the `src/components/__tests__/` directory.
-   Run tests using:
    ```bash
    npm test
    # or
    yarn test
    ```
