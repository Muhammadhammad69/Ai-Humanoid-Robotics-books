# Implementation Plan: ROS 2 Robotic Nervous System Module Content

**Feature Branch**: `003-ros2-robot-nervous-system`
**Created**: 2025-12-07
**Status**: Draft

## Technical Context

This plan outlines the process for generating the complete content for "Module 1 - The Robotic Nervous System (ROS 2)" of the Physical AI & Humanoid Robotics book. The content will be textbook-quality, beginner-friendly yet technically precise, and strictly adhere to official sources. The generation process will leverage Playwright MCP for web scraping, Context7 MCP for documentation integration, and GitHub MCP for version control and deployment.

## Constitution Check

- **I. Accuracy**: Achieved through primary source verification (ROS 2 Docs, URDF Reference, rclpy docs) using Playwright MCP and technical validation steps. Real-world examples will be extracted and verified.
- **II. Clarity**: Content will be generated in a formal, textbook-like style, targeting developers familiar with AI tools, but also accessible to beginners, as specified in the feature requirements.
- **III. Reproducibility**: All code snippets will be extracted from official sources and validated for executability. Specs are traceable to Spec-Kit Plus artifacts (this plan and the feature spec).
- **IV. Rigor**: Open-source tools (ROS 2, rclpy) are central to the content. Citations to official documentation and GitHub repos will be provided where appropriate.

## 1. Scope of Work

Module 1 content generation will include:
- Full textbook-quality explanations for each chapter.
- Comprehensive coverage of ROS 2 foundational concepts (Nodes, Topics, Services).
- Detailed guidance on `rclpy` usage and Python integration.
- Thorough explanation of URDF for humanoid robots, including valid tag usage.
- Practical hands-on examples and exercises integrated within chapters.
- Descriptions for diagrams (text descriptions only; no image generation).
- Verification against official references and sources.

The scope is strictly limited to Module 1, comprising the following chapters:
1.  Introduction to ROS 2
2.  ROS 2 Nodes, Topics, and Services
3.  Bridging Python Agents with `rclpy`
4.  URDF for Humanoid Robots
5.  Hands-On Projects & Exercises

Modules 2–4 content is explicitly out of scope for this plan.

## 2. Authoritative Source Strategy (Playwright MCP Powered)

Playwright MCP will be used as the primary tool for extracting and validating information from authoritative online sources.

### Mandatory Official Websites & Scraping Strategy

- **ROS 2 Documentation (docs.ros.org)**:
    - **URLs to scrape**:
        - `https://docs.ros.org/en/{ROS2_DISTRO}/Concepts.html` (for core concepts)
        - `https://docs.ros.org/en/{ROS2_DISTRO}/Tutorials.html` (for example code and commands)
        - Specific pages for "nodes", "topics", "services", "parameters", "actions".
    - **Search Queries**: `site:docs.ros.org/en/{ROS2_DISTRO} {concept_name} {keyword}` (e.g., "site:docs.ros.org/en/humble nodes python").
    - **Information Extraction**: Verbatim extraction of definitions, command-line usage, API descriptions, and code examples. Summarization of conceptual overviews and background information.
- **ROS 2 Tutorials**:
    - **URLs to scrape**: `https://docs.ros.org/en/{ROS2_DISTRO}/Tutorials/` (specific tutorials for publisher/subscriber, service client/server, etc.).
    - **Search Queries**: `site:docs.ros.org/en/{ROS2_DISTRO}/Tutorials {tutorial_name}`.
    - **Information Extraction**: Extract runnable code examples, step-by-step instructions, and command outputs.
- **`rclpy` API reference**:
    - **URLs to scrape**: `https://docs.ros.org/en/{ROS2_DISTRO}/p/rclpy/` (and subpages for specific modules/classes).
    - **Search Queries**: `site:docs.ros.org/en/{ROS2_DISTRO}/p/rclpy {module_or_class_name}`.
    - **Information Extraction**: Verbatim extraction of function signatures, class descriptions, and usage examples.
- **URDF XML specification**:
    - **URLs to scrape**: `http://wiki.ros.org/urdf` and related links to XML schema definitions.
    - **Search Queries**: `site:wiki.ros.org urdf xml specification tags`
    - **Information Extraction**: Verbatim extraction of element/attribute definitions (`<link>`, `<joint>`, `<inertial>`, `<visual>`, `<collision>`), their properties, and structural rules.
- **REP (ROS Enhancement Proposals)**:
    - **URLs to scrape**: `https://www.ros.org/reps/` (specific REP documents relevant to core ROS 2 design).
    - **Search Queries**: `site:ros.org/reps {rep_number} {topic}`.
    - **Information Extraction**: Summarization of design decisions, rationale, and standards.
- **Gazebo/ROS integration notes (when relevant)**:
    - **URLs to scrape**: Specific Gazebo documentation pages or ROS-Gazebo tutorials related to URDF parsing or simulation.
    - **Search Queries**: `site:gazebosim.org {topic} ros urdf`.
    - **Information Extraction**: Relevant snippets on URDF-Gazebo compatibility, physics, or sensor integration.

### Validation of Scraped Data

- **Syntactic Validation**: Use regex and structural parsing to ensure extracted code, commands, and XML conform to expected syntax (e.g., Python syntax checker, XML schema validator for URDF).
- **Semantic Validation (Cross-referencing)**: Compare definitions and API usage across multiple official sources. Flag inconsistencies for manual review.
- **Code Executability Check**: For Python/`rclpy` examples, attempt to run them in a sandboxed ROS 2 environment (if feasible within Playwright's capabilities, or mark for a separate validation step). For ROS 2 CLI commands, verify their existence and basic usage patterns.
- **Summarization Quality**: Evaluate summaries for accuracy and conciseness, ensuring no critical information is lost.

## 3. Documentation Integration Workflow (Context7 MCP)

Context7 MCP will manage the integration of generated content into the Docusaurus-based book structure, ensuring consistency and adherence to the defined chapter template.

### Integration Steps

1.  **Read Existing Placeholders**: Context7 MCP will read the initial module-1 placeholder files (if they exist) to identify the target locations for content insertion.
2.  **Content Writing**:
    -   Generated content (Markdown) will be written into the respective chapter files (`intro.md`, `nodes-topics-services.md`, `rclpy-bridge.md`, `urdf-humanoids.md`, `hands-on-projects.md`).
    -   The tool will enforce the mandatory chapter template structure:
        -   `# [Chapter Title]`
        -   `## Overview`
        -   `## Learning Objectives`
        -   `## Key Concepts`
        -   `## Technical Deep Dive`
        -   `## Code Examples`
    -   Content for `Learning Objectives` and `Key Concepts` will be derived from the extracted information and formatted appropriately.
3.  **Formatting Validation & Organization**:
    -   **Code Blocks**: Ensure `python` or `xml` (for URDF) fencing for code examples.
    -   **Diagrams**: Text descriptions for diagrams will be inserted directly into the Markdown. If a diagram *image* were to be generated (not in current scope), its text description placeholder would be saved in a designated `/assets/` subdirectory within the module.
    -   **Exercises**: Markdown formatting for exercises will be applied consistently.
4.  **Sidebar Integration & Frontmatter**:
    -   Context7 MCP will read `sidebars.ts` to understand the existing book structure.
    -   It will ensure that the new chapter files are correctly linked and do not break the sidebar navigation.
    -   Frontmatter (e.g., `id`, `title`, `sidebar_position`) will be generated or updated for each chapter, adhering to Docusaurus conventions.
    -   Headings will be checked against the specified structure to prevent inconsistencies.
5.  **Broken Link Checking**: After content insertion, Context7 MCP will perform an internal link check within the generated module to ensure all cross-references are valid.

## 4. Research → Drafting → Validation Pipeline

This pipeline ensures the accuracy and quality of the generated content, minimizing hallucinations.

### Step 1 — Research (Playwright MCP)

-   **Objective**: Gather raw, validated information from official sources.
-   **Process**:
    -   Playwright MCP will navigate to the URLs identified in Section 2.
    -   It will execute targeted searches and scrape relevant text, code snippets, and command-line outputs.
    -   Initial syntactic and semantic validation will occur here (e.g., basic Python/XML parsing, cross-referencing definitions).
-   **Output**: Structured data containing definitions, API usage, code examples, ROS 2 architecture descriptions, and command outputs, along with source URLs.

### Step 2 — Content Drafting

-   **Objective**: Generate the chapter content using the researched and validated material.
-   **Process**:
    -   An LLM (e.g., Gemini) will consume the structured research data.
    -   The LLM will be instructed to synthesize this information into the specified chapter structure, adhering to the writing style and tone (formal, precise, beginner-friendly).
    -   It will craft overviews, learning objectives, key concepts, technical deep dives, and integrate code examples and diagram descriptions.
    -   The LLM will be explicitly instructed to *only* use information from the provided research data to prevent hallucination.
-   **Output**: Raw Markdown content for each chapter of Module 1.

### Step 3 — Technical Validation

-   **Objective**: Verify the technical accuracy and executability of generated code and commands.
-   **Process**:
    -   **Code Examples**: All Python (`rclpy`) code snippets will be extracted from the drafted content. A sandboxed ROS 2 environment will be used to attempt execution of these snippets. Any execution failures or unexpected outputs will be flagged.
    -   **URDF Snippets**: URDF XML snippets will be extracted and validated against a formal URDF XML schema. Tools like `urdf_parser_py` or `check_urdf` (if available and compatible) will be used.
    -   **ROS Commands**: Command-line interface (`ros2`) commands will be checked for syntactic correctness and a sample execution will be attempted in the sandboxed environment (e.g., `ros2 topic list`, `ros2 node info`).
    -   **Fact-Checking**: Key technical assertions in the drafted content will be cross-referenced against the original scraped data and, if necessary, against live official documentation to ensure accuracy.
-   **Output**: A validation report detailing passes/failures and identified technical inaccuracies.

### Step 4 — Formatting Validation (Context7 MCP)

-   **Objective**: Ensure the generated Markdown content conforms to Docusaurus requirements and overall formatting standards.
-   **Process**:
    -   Context7 MCP will initiate a Docusaurus build process for the updated book.
    -   It will parse build logs for any Markdown errors, broken links, or inconsistencies in frontmatter/headings that could cause build failures or display issues.
    -   Identified errors will be reported, and if possible, automated fixes will be applied (e.g., correcting link paths, ensuring consistent heading levels).
    -   The internal link checking (from Section 3) is a part of this step.
-   **Output**: A formatting validation report and, if successful, a clean Docusaurus build.

## 5. Chapter-by-Chapter Implementation Strategy

This section details the specific approach for generating content for each of the five chapters in Module 1.

### 1. Introduction to ROS 2

-   **Required Research Sources**: ROS 2 Concepts documentation (especially "What is ROS 2?", "ROS 2 Architecture"), beginner ROS 2 tutorials.
-   **Required Diagrams (Text Descriptions)**: High-level ROS 2 architectural overview (nodes, topics, services, client libraries), communication graph.
-   **Required Code Examples**: Simple `ros2 run` commands for basic talker/listener (conceptual, not full code for this intro chapter).
-   **Required Exercises**: Conceptual questions about ROS 2 benefits, basic terminology matching.
-   **Validation Steps**: Verify accuracy of architectural descriptions against `docs.ros.org/en/{ROS2_DISTRO}/Concepts.html`.
-   **Technical Review Criteria**: Clarity of explanation, correct terminology, appropriate level of detail for an introduction.

### 2. ROS 2 Nodes, Topics, and Services

-   **Required Research Sources**: ROS 2 Nodes, Topics, Services documentation and tutorials (`docs.ros.org/en/{ROS2_DISTRO}/Tutorials/` for Python `rclpy` examples).
-   **Required Diagrams (Text Descriptions)**: Node communication diagram (publisher to subscriber), service client-server interaction.
-   **Required Code Examples**:
    -   Python `rclpy` code for a basic publisher.
    -   Python `rclpy` code for a basic subscriber.
    -   Python `rclpy` code for a basic service server.
    -   Python `rclpy` code for a basic service client.
    -   Associated `ros2 cli` commands (`ros2 run`, `ros2 topic list`, `ros2 topic echo`, `ros2 service list`, `ros2 service call`).
-   **Required Exercises**: Modify publisher/subscriber rates, create a simple service, observe topic/service data with `ros2 cli`.
-   **Validation Steps**: Execute all `rclpy` code and `ros2 cli` commands in a sandboxed environment. Verify explanations match `docs.ros.org`.
-   **Technical Review Criteria**: Correct implementation of ROS 2 communication patterns, accurate `rclpy` API usage, clear explanations of `qos_profiles`.

### 3. Bridging Python Agents with `rclpy`

-   **Required Research Sources**: `rclpy` API reference (`docs.ros.org/en/{ROS2_DISTRO}/p/rclpy/`), advanced `rclpy` tutorials (e.g., parameters, custom messages, actions).
-   **Required Diagrams (Text Descriptions)**: Python agent interacting with `rclpy` node, custom message definition flow.
-   **Required Code Examples**:
    -   Python `rclpy` node with parameters.
    -   Python `rclpy` node using custom messages (publisher/subscriber with generated `msg` type).
    -   Basic ROS 2 Action client/server in Python.
-   **Required Exercises**: Create a custom message, implement a simple parameter server, build a basic action.
-   **Validation Steps**: Execute all `rclpy` code. Verify custom message generation and usage.
-   **Technical Review Criteria**: Correct `rclpy` API usage for advanced features, proper handling of custom messages, clear explanation of Python-ROS 2 integration.

### 4. URDF for Humanoid Robots

-   **Required Research Sources**: URDF XML specification (`wiki.ros.org/urdf`), examples of humanoid URDFs (e.g., `simple_humanoid.urdf` used previously).
-   **Required Diagrams (Text Descriptions)**: Link-joint tree structure, coordinate frames, visual/collision geometry.
-   **Required Code Examples**:
    -   Complete URDF for a simple 2-link humanoid robot (base, link1, link2, with joints, inertial, visual, collision).
    -   Fragments demonstrating `<material>`, `<limit>`, `<origin>` tags.
-   **Required Exercises**: Modify visual properties, change joint limits, add a simple sensor link.
-   **Validation Steps**: Validate generated URDF against XML schema and `check_urdf` tool. Verify explanations match `wiki.ros.org/urdf`.
-   **Technical Review Criteria**: Syntactic correctness of URDF, accurate explanation of each tag's purpose, use of a consistent humanoid example.

### 5. Hands-On Projects & Exercises

-   **Required Research Sources**: ROS 2 project tutorials, common robotics tasks (e.g., teleoperation, basic navigation).
-   **Required Diagrams (Text Descriptions)**: System architecture for a mini-project (e.g., teleop controller node communicating with robot joint commander node).
-   **Required Code Examples**:
    -   Python script for teleoperating the 2-link humanoid (e.g., move joint based on keyboard input).
    -   Python script for publishing joint states from a simulated or actual robot to ROS 2 topics.
    -   (Optional, if complexity permits) Simple `rviz` configuration file description.
-   **Required Exercises**: Implement a basic PID controller for a joint, integrate a simulated sensor, visualize in `rviz`.
-   **Validation Steps**: Execute all project code in a sandboxed ROS 2 environment. Ensure exercises are solvable with provided information.
-   **Technical Review Criteria**: Project feasibility for beginners, clear instructions, runnable code, effective demonstration of concepts from previous chapters.

## 6. Quality Controls

### Accuracy Controls

-   **No Invented Content**: All ROS commands, APIs, and technical assertions will be sourced from the Playwright MCP research step. Any deviation will be flagged.
-   **Code Validation**: All code examples (Python `rclpy`, `ros2 cli` commands) will undergo execution tests in a sandboxed ROS 2 environment. Expected outputs will be compared against actual outputs.
-   **URDF Validation**: URDF snippets will be formally validated against the official XML schema for correctness, including all tags (`<link>`, `<joint>`, `<inertial>`, `<visual>`, `<collision>`).
-   **Version Specificity**: Content will strictly adhere to the specified ROS 2 distributions (Foxy/Humble), and any version-specific differences will be explicitly noted.

### Structure Controls

-   **Chapter Template Adherence**: Context7 MCP will enforce the mandatory chapter structure, ensuring all sections (Overview, Learning Objectives, Key Concepts, Technical Deep Dive, Code Examples) are present and correctly formatted. Missing sections will trigger an error.
-   **Official References**: All external links and citations within the generated content will be programmatically verified to point to official ROS 2, URDF, or `rclpy` documentation.
-   **Consistency**: Terminology will be checked for consistency across chapters. The use of a unified "simple 2-link robot" example for URDF will be validated.
-   **Markdown Linting**: The final Markdown files will be subjected to a Markdown linter to catch syntax errors, inconsistent formatting, and potential rendering issues in Docusaurus.

## 7. MCP Automation Strategy

This section outlines which tasks will be automated by the MCPs and which will require human oversight.

### Playwright MCP Automations:

-   **Live Scraping**: Automated navigation to specified URLs and extraction of textual content, code snippets, and command examples.
-   **Collecting Official Docs**: Programmatic gathering of data from `docs.ros.org`, `wiki.ros.org/urdf`, `rclpy` API reference, and REP documents.
-   **Pulling Real Code Examples**: Automated identification and extraction of Python `rclpy` and `ros2 cli` code blocks from tutorials and documentation.
-   **Verifying API Names and Parameters**: Automated comparison of extracted API calls and parameters against the `rclpy` API reference.

### Context7 MCP Automations:

-   **Create & Update Markdown Files**: Automated creation of new `.md` files for chapters and seamless updating of existing ones with generated content.
-   **Insert Generated Content**: Automated population of chapter template sections with text and code.
-   **Run Builds and Report Build Errors**: Triggering Docusaurus build processes and parsing build logs to identify and report errors (e.g., Markdown syntax, broken internal links).
-   **Fix Broken Paths or Headers**: Automated correction of common formatting issues, internal link paths, and ensuring consistent heading levels within Markdown files.
-   **Sidebar Integrity Check**: Verification that changes to chapter files do not disrupt the Docusaurus sidebar structure (`sidebars.ts`).

### GitHub MCP Automations:

-   **Commit Finalized Module 1**: Automated creation of git commits containing the generated and validated Module 1 content files.
-   **Push to Repository**: Automated pushing of the feature branch with new content to the remote GitHub repository.
-   **Trigger GitHub Pages Deployment (Optional Later)**: (Out of scope for this plan but noted as a potential future automation) Automated triggering of a GitHub Pages deployment workflow once content is approved and merged.

### Human Oversight Tasks:

-   **Review of Generated Content**: Critical human review of drafted content for overall coherence, clarity, tone, and pedagogical effectiveness.
-   **Resolution of Validation Flags**: Human intervention to resolve any inconsistencies flagged during technical or formatting validation that cannot be auto-corrected.
-   **Diagram Design**: Human creation of actual visual diagrams based on the text descriptions provided in the content.
-   **Final Approval**: Human sign-off before content is committed and merged.
-   **Strategic Adjustments**: Human decision-making for any unforeseen challenges or significant deviations from the plan.

## 8. Final Deliverables From This Plan

This `/sp.plan` generates the following artifacts:

-   **Full Pipeline Description**: As detailed in Section 4 ("Research → Drafting → Validation Pipeline").
-   **Research Map**: Outlined in Section 2 ("Authoritative Source Strategy"), detailing URLs, search queries, and extraction methods.
-   **Chapter-by-Chapter Breakdown**: Detailed in Section 5 ("Chapter-by-Chapter Implementation Strategy"), specifying sources, diagrams, code, exercises, and validation for each chapter.
-   **Validation Checklist**: Implicitly created through the "Quality Controls" (Section 6) and "Technical Validation" (Section 4, Step 3) which will guide the creation of a separate validation checklist during the `/sp.tasks` phase.
-   **Step-by-Step Flow for `/sp.tasks`**: The detailed steps within each section, particularly the "Research → Drafting → Validation Pipeline" and "Chapter-by-Chapter Implementation Strategy", serve as direct input for creating granular tasks.
-   **Clear Definition of MCP Usage**: Explicitly defined throughout Sections 2, 3, and 7, demonstrating how Playwright MCP, Context7 MCP, and GitHub MCP will be utilized to produce Module 1 content.