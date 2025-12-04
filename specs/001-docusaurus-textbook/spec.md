# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `001-docusaurus-textbook`
**Created**: 2025-12-04
**Status**: Draft
**Input**: User description: "AI/Spec-Driven Book Creation for Physical AI & Humanoid Robotics Textbook"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learning Journey (Priority: P1)

As a student, I want a navigable Docusaurus site with 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA/Capstone) covering the 13-week course, so I can learn embodied intelligence step-by-step.

**Why this priority**: This is the core value proposition - delivering educational content in an accessible, structured format. Without this, the textbook has no purpose. This represents the Minimum Viable Product (MVP).

**Independent Test**: Can be fully tested by navigating through all 4 modules via the sidebar, reading content for each week (1-13), and verifying that all sections are accessible without errors. Delivers complete educational content that students can learn from independently.

**Acceptance Scenarios**:

1. **Given** a student visits the textbook homepage, **When** they view the sidebar navigation, **Then** they see 4 clearly labeled modules: "Module 1: Robotic Nervous System (ROS 2)", "Module 2: Digital Twin (Gazebo & Unity)", "Module 3: AI-Robot Brain (NVIDIA Isaac)", and "Module 4: Vision-Language-Action (VLA)"
2. **Given** a student expands Module 1 in the sidebar, **When** they review the topics, **Then** they see sections for Nodes, Topics, Services, rclpy, and URDF with clear descriptions
3. **Given** a student opens any chapter, **When** they read the content, **Then** they find explanations, code snippets, and examples that teach the concept without requiring external resources
4. **Given** a student completes Module 1, **When** they move to Module 2, **Then** the content builds on previous knowledge without gaps
5. **Given** a student accesses the site on mobile, **When** they navigate through modules, **Then** the sidebar and content adapt responsively to the screen size
6. **Given** a student uses the search function, **When** they search for "URDF", **Then** they see relevant results from multiple modules with context snippets
7. **Given** a student prefers dark mode, **When** they toggle the theme, **Then** all content renders with dark theme colors without readability issues

---

### User Story 2 - Instructor Course Management (Priority: P2)

As an instructor, I want embedded hardware requirements (e.g., $700 Economy Jetson Kit table) and assessments (ROS packages, simulations, capstone), so I can assign practical labs and guide students through the 13-week course.

**Why this priority**: Instructors need structured course materials to teach effectively. This story adds pedagogical value through assessments, hardware guidance, and weekly structure - critical for classroom adoption but not required for self-learners.

**Independent Test**: Can be tested by reviewing the "Course Overview" section for the 13-week breakdown, verifying the hardware requirements table displays accurate pricing ($700 Economy Jetson Kit breakdown), and confirming all 4 assessment projects are documented with clear deliverables.

**Acceptance Scenarios**:

1. **Given** an instructor views the course overview page, **When** they review the weekly breakdown, **Then** they see Weeks 1-13 with specific topics, learning objectives, and time estimates for each week
2. **Given** an instructor reviews hardware requirements, **When** they view the Economy Jetson Kit table, **Then** they see exact prices: Jetson Orin Nano ($249), RealSense D435i ($349), ReSpeaker ($69), peripherals/power (~$30-100), totaling ~$700
3. **Given** an instructor plans lab assignments, **When** they review assessment descriptions, **Then** they find 4 projects: (1) ROS 2 pub/sub package, (2) Gazebo simulation with sensors, (3) Isaac Sim sim-to-real transfer, (4) VLA autonomous humanoid capstone
4. **Given** an instructor advises students on hardware purchases, **When** they compare options, **Then** they see comparison tables for Digital Twin Workstation (RTX 4070+), Edge Kit ($700), Robot Lab (Unitree Go2 $1800-3000, G1 $16k), and Cloud-Native (~$205/quarter)
5. **Given** an instructor assigns Module 3, **When** they review the sim-to-real section, **Then** they see the "Latency Trap Warning" prominently displayed explaining why cloud robot control is dangerous
6. **Given** an instructor prepares course logistics, **When** they review learning outcomes, **Then** they see 6 clearly defined measurable outcomes aligned with the 4 modules

---

### User Story 3 - Interactive Engaged Reading (Priority: P3)

As a reader, I want interactive MDX elements like code snippets, warnings for latency traps, and "why Physical AI matters" sections, so the content is engaging and actionable.

**Why this priority**: Interactive elements enhance learning engagement and retention but are not blocking for basic content consumption. This story adds polish and pedagogical effectiveness to the MVP content.

**Independent Test**: Can be tested by opening any chapter and verifying the presence of: (1) syntax-highlighted code blocks with copy buttons, (2) warning callouts for latency traps in sim-to-real sections, (3) "Why Physical AI Matters" section in the introduction, and (4) interactive diagrams or tables where applicable.

**Acceptance Scenarios**:

1. **Given** a reader views a Python code example in Module 1, **When** they hover over the code block, **Then** they see a "Copy" button that copies the code to clipboard when clicked
2. **Given** a reader reaches a sim-to-real section, **When** they view content discussing cloud-edge communication, **Then** they see a prominently styled warning callout (yellow/orange background) explaining the latency trap with specific latency ranges (50-200ms+)
3. **Given** a reader visits the introduction page, **When** they read the "Why Physical AI Matters" section, **Then** they see a compelling explanation of bridging digital AI to physical humanoids with real-world applications
4. **Given** a reader views the hardware requirements table, **When** they scan the Economy Jetson Kit pricing, **Then** the table is formatted with clear columns (Component, Price, Source) and includes links to vendor pages
5. **Given** a reader reviews Module 4 VLA examples, **When** they see the "Clean the room" natural language command, **Then** the explanation shows the breakdown: Whisper transcription → LLM planning → ROS action execution with clear visual flow
6. **Given** a reader views URDF/SDF examples, **When** they see XML code snippets, **Then** the syntax highlighting differentiates tags, attributes, and values for readability

---

### User Story 4 - Public Deployment & Access (Priority: P4)

As a deployer, I want automatic GitHub Pages deployment, so the book is publicly accessible without paywalls (login optional for bonuses like RAG chatbot, Urdu translation, and personalization).

**Why this priority**: Deployment is essential for making the textbook accessible, but it's a technical enabler rather than core content. This story ensures the textbook reaches users but doesn't add educational value itself.

**Independent Test**: Can be tested by: (1) committing changes to the repository, (2) verifying the GitHub Actions workflow runs successfully, (3) accessing the deployed site at the GitHub Pages URL, and (4) confirming the site is publicly accessible without authentication.

**Acceptance Scenarios**:

1. **Given** a content author pushes changes to the main branch, **When** the commit completes, **Then** a GitHub Actions workflow triggers automatically and builds the Docusaurus site
2. **Given** the GitHub Actions workflow completes, **When** a user visits the GitHub Pages URL, **Then** they see the updated content within 5 minutes of the commit
3. **Given** an anonymous user visits the deployed site, **When** they navigate through all modules, **Then** they can read all content without encountering any login prompts or paywalls
4. **Given** a user with special features enabled (e.g., RAG chatbot, Urdu translation), **When** they visit the site, **Then** they see placeholders or disabled states for these bonus features with messaging: "Sign up to unlock"
5. **Given** the repository includes a LICENSE file, **When** a user views the deployed site footer, **Then** they see "Licensed under CC-BY-SA 4.0 (content) + MIT (code)" with links to license texts
6. **Given** the site is deployed, **When** search engines index the site, **Then** all pages are crawlable and indexable (no robots.txt restrictions, proper meta tags)

---

### Edge Cases

- **What happens when a user has JavaScript disabled?** The site should degrade gracefully - core content (Markdown) remains readable, but interactive features (sidebar collapse, code copy buttons, search) may not function.
- **What happens when a module has no content yet?** The sidebar shows the module title and placeholder text: "Content coming soon" with expected availability date.
- **What happens when hardware prices change significantly?** The hardware requirements page includes a "Last Updated: [DATE]" timestamp and a note: "Prices verified as of [DATE]. Check vendor sites for current pricing."
- **What happens when a user accesses an old/bookmarked URL after site restructure?** The site includes redirects for common old paths and a custom 404 page with search functionality and sitemap links.
- **What happens when the GitHub Pages build fails?** The repository includes a notification mechanism (e.g., GitHub Actions status badge in README) and the previous successful build remains live until the issue is resolved.
- **What happens when a student tries to run code examples on unsupported platforms (e.g., Windows for ROS 2)?** Each code example includes a platform compatibility note (Linux, macOS, Windows/WSL2) and links to setup guides.

## Requirements *(mandatory)*

### Functional Requirements

**Content Structure**

- **FR-001**: Site MUST contain exactly 4 modules organized hierarchically in the sidebar: Module 1 (Robotic Nervous System - ROS 2), Module 2 (Digital Twin - Gazebo & Unity), Module 3 (AI-Robot Brain - NVIDIA Isaac), Module 4 (Vision-Language-Action - VLA)
- **FR-002**: Module 1 MUST include sections covering: ROS 2 nodes, topics, services, actions, rclpy programming, and URDF robot descriptions
- **FR-003**: Module 2 MUST include sections covering: Physics simulation fundamentals, Gazebo setup and worlds, Unity simulation alternative, sensor simulation (LiDAR, Depth cameras, IMUs), URDF/SDF model formats
- **FR-004**: Module 3 MUST include sections covering: NVIDIA Isaac Sim installation and setup, synthetic data generation, Visual SLAM (VSLAM), Nav2 navigation stack, sim-to-real transfer techniques with latency considerations
- **FR-005**: Module 4 MUST include sections covering: Whisper voice command transcription, LLM-based planning (natural language to actions), ROS action integration, capstone autonomous humanoid project guide

**Course Information**

- **FR-006**: Site MUST include a Course Overview page with: quarter description, "Why Physical AI Matters" motivation section, 6 measurable learning outcomes, weekly breakdown for Weeks 1-13, and 4 assessment descriptions
- **FR-007**: Site MUST document 4 assessments: (1) ROS 2 publisher/subscriber package, (2) Gazebo simulation with integrated sensors, (3) Isaac Sim sim-to-real deployment, (4) VLA autonomous humanoid capstone project
- **FR-008**: Learning outcomes MUST cover: Understanding ROS 2 architecture, Creating digital twins with physics simulation, Generating and using synthetic data, Implementing sim-to-real workflows, Integrating VLA models with robots, Deploying AI on edge devices

**Hardware Requirements**

- **FR-009**: Site MUST display a Hardware Requirements page with 4 configuration options: Digital Twin Workstation, Economy Jetson Kit ($700), Robot Lab Hardware, Cloud-Native Alternative
- **FR-010**: Economy Jetson Kit section MUST include an exact pricing table with components: Jetson Orin Nano Developer Kit ($249), RealSense D435i ($349), ReSpeaker Mic Array ($69), Power supply + cables + SD card (~$30-100), with total ~$700
- **FR-011**: Digital Twin Workstation section MUST specify: RTX 4070 or better GPU, 32GB+ RAM, 1TB+ SSD, Ubuntu 22.04 LTS recommendation
- **FR-012**: Robot Lab Hardware section MUST list: Unitree Go2 (quadruped, $1800-3000), Unitree G1 (humanoid, $16,000), with clear indication these are optional for advanced learners
- **FR-013**: Cloud-Native Alternative section MUST display cost calculation: AWS g5.xlarge (A10G GPU) at $1.006/hour × 5 hours/week × 10 weeks ≈ $205/quarter with exact pricing sources
- **FR-014**: Hardware Requirements page MUST include a "Last Updated" timestamp visible at the top of the page

**Interactive Elements & Safety**

- **FR-015**: Every sim-to-real section (modules 3 and 4) MUST display a "Latency Trap Warning" callout explaining: Network latency (50-200ms+) makes cloud control unsafe for real robots, acceptable only for simulation/high-level planning, edge deployment required for real-time control
- **FR-016**: All code examples MUST be displayed in syntax-highlighted code blocks with language labels (Python, XML, Bash, etc.)
- **FR-017**: Code blocks MUST include a "Copy to Clipboard" button that appears on hover or focus
- **FR-018**: Site MUST include a global search function accessible from the navigation bar that searches all module content
- **FR-019**: Site MUST support dark mode toggle with persistent preference (stored in browser local storage)
- **FR-020**: Site MUST be fully responsive and usable on mobile devices (320px width minimum) with collapsible sidebar

**Deployment & Access**

- **FR-021**: Site MUST deploy automatically via GitHub Actions workflow triggered on commits to the main branch
- **FR-022**: Site MUST be publicly accessible at a GitHub Pages URL without requiring authentication
- **FR-023**: Site MUST display license information in the footer: "Content: CC-BY-SA 4.0 | Code: MIT" with hyperlinks to full license texts
- **FR-024**: Site MUST NOT include paywalls, login requirements, or content gating for the core 4 modules and course information
- **FR-025**: Site MUST include placeholders for future bonus features (RAG chatbot, Urdu translation, personalization) with "Sign up to unlock" messaging (features themselves not implemented in this spec)

**Content Quality & Accuracy**

- **FR-026**: All hardware prices (Jetson, RealSense, etc.) MUST be verified as accurate for December 2025 and include source vendor links
- **FR-027**: All technical specifications (Jetson Orin Nano 10 TOPS INT8, RealSense D435i depth range, etc.) MUST match official vendor documentation
- **FR-028**: Site MUST NOT include any examples or tutorials showing cloud-controlled real robots (cloud training/simulation is allowed)
- **FR-029**: Every tutorial involving robot deployment MUST explicitly state whether it applies to simulation or real hardware

### Non-Functional Requirements

**Performance**

- **NFR-001**: Site MUST load the homepage within 3 seconds on standard broadband (25 Mbps) connections
- **NFR-002**: Navigation between pages MUST feel instant (<500ms) due to static site generation
- **NFR-003**: Search results MUST appear within 1 second of query submission

**Accessibility**

- **NFR-004**: Site MUST meet WCAG 2.1 Level AA accessibility standards (keyboard navigation, screen reader support, color contrast)
- **NFR-005**: All images and diagrams MUST include descriptive alt text
- **NFR-006**: Site MUST be usable without JavaScript (content readable, navigation functional via standard HTML links)

**Maintainability**

- **NFR-007**: Content MUST be authored in Markdown/MDX format stored in a `/content/` directory (per Constitution Principle I: Single Source of Truth)
- **NFR-008**: Site structure MUST support easy addition of new modules or chapters without code changes (configuration-driven sidebar)

**Compatibility**

- **NFR-009**: Site MUST render correctly in latest versions of Chrome, Firefox, Safari, and Edge browsers
- **NFR-010**: Site MUST be mobile-responsive with breakpoints for phone (320px+), tablet (768px+), and desktop (1024px+)

### Key Entities

- **Module**: A major educational unit containing multiple chapters/sections. Attributes: title, description, order (1-4), icon/emoji identifier. Relationships: Contains many Chapters.
- **Chapter**: A single topic or lesson within a Module. Attributes: title, content (MDX), module assignment, order within module. Relationships: Belongs to one Module, contains many Code Examples.
- **Hardware Configuration**: A recommended hardware setup for students. Attributes: name (e.g., "Economy Jetson Kit"), components list, total cost, last updated date. Relationships: Contains many Hardware Components.
- **Hardware Component**: A specific piece of equipment. Attributes: name (e.g., "Jetson Orin Nano"), price, vendor, vendor URL, specifications. Relationships: Belongs to one or more Hardware Configurations.
- **Assessment**: A graded project for students. Attributes: title, description, module association, deliverables, grading criteria. Relationships: Associated with one Module.
- **Week**: A unit in the 13-week course timeline. Attributes: week number (1-13), topics covered, learning objectives, assigned readings/labs. Relationships: References multiple Modules/Chapters.

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can navigate from the homepage to any specific chapter in Module 3 within 15 seconds without prior training
- **SC-002**: 95% of code examples can be copied and executed successfully on the specified platform (Linux/WSL2 for ROS 2) without modifications
- **SC-003**: Hardware Requirements page enables students to purchase the correct Economy Jetson Kit components within 30 minutes by providing exact vendor links and verified prices
- **SC-004**: All 4 modules are fully accessible and readable on mobile devices (verified via responsive design testing on 320px-768px widths)
- **SC-005**: Site loads and renders the complete homepage (including sidebar navigation) in under 3 seconds on standard broadband connections (25 Mbps)
- **SC-006**: Search functionality returns relevant results within 1 second for 95% of queries related to course topics (ROS 2, Isaac Sim, URDF, etc.)
- **SC-007**: Anonymous users can access and read 100% of the 4 modules and course information without encountering login prompts or paywalls
- **SC-008**: Instructors can extract a complete 13-week syllabus (weekly topics + assessments) from the Course Overview page in under 10 minutes
- **SC-009**: Every sim-to-real tutorial includes a visible Latency Trap Warning within the first 2 paragraphs of the section
- **SC-010**: GitHub Pages deployment completes within 5 minutes of committing content changes, making updates publicly visible
- **SC-011**: Dark mode toggle works on all pages and persists across browser sessions (verified via local storage check)
- **SC-012**: Site receives zero accessibility violations when tested with automated tools (aXe, Lighthouse) for WCAG 2.1 Level AA compliance

## Assumptions

1. **Platform Assumption**: Students will primarily use Ubuntu 22.04 LTS or WSL2 on Windows for hands-on labs, as ROS 2 has best support on these platforms.
2. **Prerequisite Knowledge**: Students have basic programming experience (Python fundamentals) and can use command-line interfaces. No prior robotics or ROS knowledge assumed.
3. **Hardware Access Timing**: Students are not required to purchase hardware (Economy Jetson Kit or robots) in Week 1; digital twin workstations and cloud alternatives allow starting immediately with simulations.
4. **Content Authoring**: All educational content (modules 1-4) will be authored in Markdown/MDX by subject matter experts or Claude Code Subagents (per Constitution Principle IX) and reviewed by humans.
5. **Deployment Frequency**: Content updates will occur frequently during initial course development (daily) and stabilize to weekly updates once the course is established.
6. **Bonus Features Scope**: RAG chatbot, Urdu translation, and personalization features are explicitly OUT OF SCOPE for this specification. This spec covers placeholders only; those features will have separate specs.
7. **Vendor Stability**: Hardware vendor prices (NVIDIA, Intel, Unitree) are assumed to remain within ±10% of stated prices for at least one academic quarter (10-13 weeks). Price verification is manual.
8. **Open Source License**: CC-BY-SA 4.0 for content and MIT for code are final license choices; no alternative licensing will be considered.
9. **GitHub Pages Hosting**: GitHub Pages is the permanent hosting solution; no migration to alternative hosting (Vercel, Netlify, custom servers) is planned.
10. **Assessment Grading**: The 4 assessments documented in this spec are descriptive only (project titles and deliverables). Detailed rubrics and autograding are out of scope.

## Dependencies

- **Docusaurus Framework**: Static site generator for building the documentation site (version TBD in planning phase)
- **GitHub Actions**: CI/CD platform for automated deployment to GitHub Pages
- **GitHub Pages**: Hosting service for the deployed static site
- **Markdown/MDX**: Content authoring format for all educational materials
- **Constitution Compliance**: This specification must comply with all 11 principles in `.specify/memory/constitution.md`, particularly:
  - Principle I (Single Source of Truth): Content in `/content/` only
  - Principle II (Specification-First): This spec exists before implementation
  - Principle III (Accuracy & Technical Truth): Prices and specs verified
  - Principle IV (Sim-to-Real First): No cloud robot control examples
  - Principle V (Cost Transparency): Exact prices with sources
  - Principle X (Open Source): CC-BY-SA 4.0 + MIT licensing
  - Principle XI (Latency Trap Rule): Warnings in sim-to-real sections

## Out of Scope

The following features are explicitly excluded from this specification and will require separate specs:

1. **RAG Chatbot**: FastAPI + Neon Postgres + Qdrant Cloud + OpenAI Agents SDK for intelligent book search
2. **Urdu Translation**: Per-chapter "Translate to Urdu" button using DeepL or NLLB models
3. **Personalization**: Per-chapter "Personalize for me" button based on user profiles (hardware/software background)
4. **Authentication System**: Better-Auth integration for user signup/signin
5. **User Profile Collection**: 4-question form collecting RTX GPU, Jetson, robot access, and programming experience
6. **Progress Tracking**: Saving user progress through modules and chapters
7. **Interactive Labs**: In-browser coding environments, Jupyter notebooks, or simulation sandboxes
8. **Video Content**: Tutorial videos, recorded lectures, or animated demonstrations
9. **Community Features**: Forums, comments, discussion boards, or user-generated content
10. **Autograding**: Automated assessment of student submissions for the 4 projects
11. **LMS Integration**: Canvas, Moodle, or Blackboard integration for institutional use
12. **Mobile App**: Native iOS/Android applications (only responsive web)
13. **Offline Mode**: Service workers or Progressive Web App (PWA) features
14. **Multi-language Support**: Languages other than English and Urdu (Urdu is a separate spec)
15. **Content Version History**: Tracking and displaying change history for chapters
