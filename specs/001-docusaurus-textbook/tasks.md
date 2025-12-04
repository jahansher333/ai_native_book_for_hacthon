# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/001-docusaurus-textbook/`
**Prerequisites**: plan.md (completed), spec.md (completed)

**Tests**:
- After Task 5: `yarn install && yarn start` (verify local dev server at http://localhost:3000)
- After each module (Tasks 7-10): `yarn start` (preview module content)
- After Task 14: `yarn build` (verify production build succeeds)
- After Task 15: `git push` (trigger GitHub Pages deployment)

**Organization**: Tasks are grouped by implementation phase and user story priority (P1-P4).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Student Learning, US2=Instructor Course Management, US3=Interactive Reading, US4=Deployment)
- Exact file paths included in descriptions

---

## Phase 1: Setup & Foundation (Tasks 1-3)

**Purpose**: Initialize Docusaurus project and core infrastructure

- [ ] **T001** [US1] Initialize Docusaurus v3 with TypeScript
  - Run: `npx create-docusaurus@latest ai_robotics_book classic --typescript`
  - Creates: `docusaurus.config.ts`, `sidebars.ts`, `package.json`, `tsconfig.json`
  - Update `package.json` to add dependencies:
    - `@docusaurus/theme-mermaid: ^3.5.0`
    - `remark-math: ^6.0.0`
    - `rehype-katex: ^7.0.0`
    - `rehype-raw: ^7.0.0`
  - Configure `docusaurus.config.ts`:
    - Title: "Physical AI & Humanoid Robotics Textbook"
    - Tagline: "From Digital Twins to Autonomous Humanoids"
    - URL: `https://[USERNAME].github.io`
    - baseUrl: `/ai_robotics_book/`
    - organizationName: `[USERNAME]`
    - projectName: `ai_robotics_book`
    - themeConfig: Enable dark mode by default, configure navbar, footer with CC-BY-SA 4.0 + MIT license
    - Markdown: Enable Mermaid (`@docusaurus/theme-mermaid`), math (`remark-math`, `rehype-katex`)
  - **Test**: `yarn install && yarn start` (verify localhost:3000 loads)

- [ ] **T002** [US1] Configure sidebars.ts with 4-module structure
  - Edit `sidebars.ts`:
    ```typescript
    const sidebars = {
      tutorialSidebar: [
        'intro',
        'course-overview',
        'hardware',
        {
          type: 'category',
          label: '🤖 Module 1: Robotic Nervous System (ROS 2)',
          collapsed: false,
          items: [
            '01-ros2/index',
            '01-ros2/nodes',
            '01-ros2/topics',
            '01-ros2/services',
            '01-ros2/actions',
            '01-ros2/rclpy',
            '01-ros2/urdf',
          ],
        },
        {
          type: 'category',
          label: '🏗️ Module 2: Digital Twin (Gazebo & Unity)',
          collapsed: false,
          items: [
            '02-gazebo-unity/index',
            '02-gazebo-unity/physics-sim',
            '02-gazebo-unity/gazebo-setup',
            '02-gazebo-unity/unity-sim',
            '02-gazebo-unity/sensors-lidar',
            '02-gazebo-unity/sensors-depth',
            '02-gazebo-unity/sensors-imu',
            '02-gazebo-unity/urdf-sdf',
          ],
        },
        {
          type: 'category',
          label: '🧠 Module 3: AI-Robot Brain (NVIDIA Isaac)',
          collapsed: false,
          items: [
            '03-isaac/index',
            '03-isaac/isaac-setup',
            '03-isaac/synthetic-data',
            '03-isaac/vslam',
            '03-isaac/nav2',
            '03-isaac/sim-to-real',
          ],
        },
        {
          type: 'category',
          label: '🗣️ Module 4: Vision-Language-Action (VLA)',
          collapsed: false,
          items: [
            '04-vla/index',
            '04-vla/whisper',
            '04-vla/llm-planning',
            '04-vla/ros-integration',
            '04-vla/capstone',
          ],
        },
      ],
    };
    export default sidebars;
    ```
  - Creates navigation structure for all 4 modules (23 chapters total)

- [ ] **T003** [P] [US3] Create custom MDX components
  - Create `src/components/HardwareTable/index.tsx`:
    - Props: `variant: 'economy' | 'workstation' | 'robot-lab' | 'cloud-native'`, `lastUpdated?: string`
    - Renders pricing table based on variant
    - Economy variant: Jetson Orin Nano ($249), RealSense D435i ($349), ReSpeaker ($69), Peripherals (~$100), **Total: $700**
    - Include vendor links (NVIDIA Store, Intel Store, Seeed Studio, Amazon)
    - Accessibility: `<table>` with `<caption>`, `<th scope="col">`, proper ARIA labels
  - Create `src/components/HardwareTable/styles.module.css`:
    - Responsive table styling
    - Vendor link hover effects
    - Dark mode support
  - Create `src/components/LatencyWarning/index.tsx`:
    - Props: `latencyRange?: string` (default: "50-200ms+"), `context?: string` (default: "cloud robot control")
    - Renders yellow/orange warning callout box
    - Text: "⚠️ LATENCY TRAP WARNING ⚠️ This architecture sends commands over the network. Network latency ({latencyRange}) makes this UNSAFE for real robots. Use this pattern ONLY for: Simulation environments (Isaac Sim, Gazebo), High-level planning (not real-time control), Data collection (not actuation). For real robots: Deploy models to edge devices (Jetson) for <10ms inference."
  - Create `src/components/LatencyWarning/styles.module.css`:
    - Background: `#FFF3CD` (light mode), `#664d03` (dark mode)
    - Border-left: 4px solid orange
    - Padding: 1rem
    - Bold title, increased font size
  - Create `src/components/PlatformNote/index.tsx`:
    - Props: `platforms: string[]`, `notes: Record<string, string>`
    - Renders platform compatibility table below code blocks
  - Create `src/components/PlatformNote/styles.module.css`
  - **Test**: Import components in a test `.mdx` file, verify rendering

---

## Phase 2: Core Content Pages (Tasks 4-6)

**Purpose**: Create intro, course overview, and hardware requirements pages (User Story 2 - Instructor)

- [ ] **T004** [US1] Create docs/intro.mdx (landing page)
  - Front matter:
    ```yaml
    ---
    id: intro
    title: Physical AI & Humanoid Robotics
    sidebar_label: Introduction
    sidebar_position: 1
    description: Learn to bridge digital AI to physical humanoids using ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action models
    keywords: [physical-ai, humanoid-robotics, ros2, isaac-sim, embodied-intelligence]
    ---
    ```
  - Content sections:
    - **Why Physical AI Matters**: Compelling explanation of bridging digital AI (LLMs, computer vision) to physical humanoids with real-world applications (warehouse automation, elder care, household assistance)
    - **What You'll Learn**: 6 learning outcomes from spec (ROS 2 architecture, digital twins, synthetic data, sim-to-real workflows, VLA integration, edge deployment)
    - **Course Structure**: Brief overview of 4 modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA) with icons
    - **Who This Is For**: Students, researchers, engineers interested in embodied AI
    - **Prerequisites**: Basic Python, command-line familiarity (no prior robotics experience required)
  - **Test**: `yarn start`, navigate to http://localhost:3000/docs/intro

- [ ] **T005** [US2] Create docs/hardware.mdx with $700 Economy Jetson Kit table
  - Front matter:
    ```yaml
    ---
    id: hardware
    title: Hardware Requirements
    sidebar_label: Hardware
    sidebar_position: 3
    description: Choose your hardware setup - Economy Jetson Kit ($700), Digital Twin Workstation, Robot Lab, or Cloud-Native
    keywords: [jetson-orin-nano, realsense-d435i, hardware-setup, edge-computing]
    ---
    ```
  - Import HardwareTable component: `import HardwareTable from '@site/src/components/HardwareTable';`
  - Content sections:
    - **Overview**: 4 hardware configuration options (Economy Kit, Workstation, Robot Lab, Cloud-Native)
    - **Economy Jetson Kit ($700)** ⭐ Recommended for Students:
      - Description: Complete edge AI setup for running models on real robots
      - `<HardwareTable variant="economy" lastUpdated="2025-12-04" />`
      - Specs: Jetson Orin Nano (10 TOPS INT8, 8GB RAM), RealSense D435i (depth 0.3-10m, 1280×720@90fps), ReSpeaker (4-mic array, USB), Power + cables + SD card
      - When to use: Deploying to real robots, edge inference, sim-to-real projects
    - **Digital Twin Workstation** (for simulation):
      - RTX 4070 or better (12GB+ VRAM), 32GB+ RAM, 1TB+ SSD, Ubuntu 22.04 LTS
      - When to use: Isaac Sim, Gazebo, training models, synthetic data generation
      - Cost: $1500-2500 (one-time)
    - **Robot Lab Hardware** (optional):
      - Unitree Go2 quadruped ($1800-3000), Unitree G1 humanoid ($16,000)
      - When to use: Advanced learners, research labs, testing on physical platforms
    - **Cloud-Native Alternative**:
      - AWS g5.xlarge (A10G GPU, 24GB VRAM): $1.006/hour
      - Cost calculation: 5 hrs/week × 10 weeks ≈ **$205/quarter**
      - When to use: No local GPU, temporary access, trying Isaac Sim
    - **Last Updated**: 2025-12-04 (timestamp visible at top)
    - **Note**: "Prices verified as of December 2025. Check vendor sites for current pricing."
  - **Test**: `yarn start`, navigate to /docs/hardware, verify $700 table renders with vendor links

- [ ] **T006** [US2] Create docs/course-overview.mdx (13-week breakdown + 4 assessments)
  - Front matter:
    ```yaml
    ---
    id: course-overview
    title: Course Overview
    sidebar_label: Course Overview
    sidebar_position: 2
    description: 13-week Physical AI course covering ROS 2, Gazebo, Isaac Sim, and VLA models with 4 hands-on projects
    keywords: [course-structure, weekly-breakdown, assessments, learning-outcomes]
    ---
    ```
  - Content sections:
    - **Quarter Overview**: 10-13 week course, 3-5 hours/week, hands-on labs
    - **Learning Outcomes** (6 points):
      1. Understand ROS 2 architecture (nodes, topics, services, actions)
      2. Create digital twins with physics simulation (Gazebo, Unity)
      3. Generate and use synthetic data for robot training (Isaac Sim)
      4. Implement sim-to-real workflows (cloud training → edge deployment)
      5. Integrate Vision-Language-Action models with robots (Whisper, LLMs, ROS)
      6. Deploy AI models on edge devices (Jetson Orin Nano) for real-time inference
    - **Weekly Breakdown** (Weeks 1-13):
      - **Week 1**: Introduction to Physical AI & ROS 2 setup
      - **Week 2**: ROS 2 nodes, topics, pub/sub pattern
      - **Week 3**: ROS 2 services, actions, rclpy programming
      - **Week 4**: URDF robot descriptions, TF2 transforms
      - **Week 5**: Gazebo simulation setup, physics engines, sensor plugins
      - **Week 6**: Digital twin sensors (LiDAR, depth cameras, IMUs), URDF/SDF models
      - **Week 7**: NVIDIA Isaac Sim installation, synthetic data generation
      - **Week 8**: Visual SLAM (VSLAM), Nav2 navigation stack
      - **Week 9**: Sim-to-real transfer techniques, deployment workflows
      - **Week 10**: Whisper voice command transcription, speech-to-text
      - **Week 11**: LLM-based planning (natural language → ROS actions), "Clean the room" example
      - **Week 12**: VLA model integration with ROS, edge deployment
      - **Week 13**: Capstone project - Autonomous humanoid with voice commands
    - **Assessments** (4 projects):
      1. **Assessment 1 (Week 4)**: ROS 2 Publisher/Subscriber Package
         - Deliverables: Python package with pub/sub nodes, launch file, README
         - Grading: Messages publish at 10Hz, subscriber logs received messages, code follows PEP 8
      2. **Assessment 2 (Week 6)**: Gazebo Simulation with Integrated Sensors
         - Deliverables: Gazebo world with robot, LiDAR + depth camera plugins, visualization in RViz
         - Grading: Sensors publish data at correct rates, robot navigates world, visualization accurate
      3. **Assessment 3 (Week 9)**: Isaac Sim Sim-to-Real Deployment
         - Deliverables: Trained model in Isaac Sim, exported ONNX weights, deployed to Jetson (simulated or real)
         - Grading: Model trains in sim, weights export successfully, inference runs on Jetson <50ms latency
      4. **Assessment 4 (Week 13)**: VLA Autonomous Humanoid Capstone
         - Deliverables: Voice-controlled robot, Whisper transcription, LLM planning, ROS action execution, video demo
         - Grading: Voice commands transcribed accurately, LLM plans actions correctly, robot executes tasks autonomously
  - **Test**: `yarn start`, navigate to /docs/course-overview

---

## Phase 3: Module Content (Tasks 7-10) - Parallelizable by Module

**Purpose**: Create all 23 chapter files for 4 modules (User Story 1 - Student Learning)

### Module 1: Robotic Nervous System (ROS 2) - 7 chapters

- [ ] **T007** [P] [US1] Create Module 1 content (docs/01-ros2/)
  - Create `docs/01-ros2/index.mdx` (Module 1 intro):
    - Front matter: `id: 01-ros2/index`, `title: Module 1: Robotic Nervous System (ROS 2)`, `sidebar_position: 1`
    - Content: Overview of ROS 2, why it's critical for Physical AI, learning objectives for this module
  - Create `docs/01-ros2/nodes.mdx`:
    - Explain ROS 2 nodes (processes that perform computation)
    - Code example: Create a minimal Python node with `rclpy`
    - Platform note: `<PlatformNote platforms={['Linux', 'macOS', 'WSL2']} />`
  - Create `docs/01-ros2/topics.mdx`:
    - Explain pub/sub pattern, topic names, message types
    - Code example: Publisher and subscriber nodes communicating via `std_msgs/String`
  - Create `docs/01-ros2/services.mdx`:
    - Explain request/response pattern, service definitions
    - Code example: Service client and server for addition
  - Create `docs/01-ros2/actions.mdx`:
    - Explain long-running tasks with feedback, action definitions
    - Code example: Fibonacci action server
  - Create `docs/01-ros2/rclpy.mdx`:
    - Explain Python client library, node lifecycle, executors
    - Code example: Multi-threaded executor
  - Create `docs/01-ros2/urdf.mdx`:
    - Explain Unified Robot Description Format, XML structure, links, joints
    - Code example: Simple 2-link robot arm URDF
    - XML syntax highlighting in code blocks
  - **Test**: `yarn start`, navigate through all 7 chapters in Module 1 sidebar

### Module 2: Digital Twin (Gazebo & Unity) - 7 chapters

- [ ] **T008** [P] [US1] Create Module 2 content (docs/02-gazebo-unity/)
  - Create `docs/02-gazebo-unity/index.mdx` (Module 2 intro)
  - Create `docs/02-gazebo-unity/physics-sim.mdx`:
    - Explain physics engines (ODE, Bullet, Simbody), collision detection, gravity
  - Create `docs/02-gazebo-unity/gazebo-setup.mdx`:
    - Installation instructions (Ubuntu 22.04 + Gazebo 11/Ignition Fortress)
    - Launch a world, spawn a robot
  - Create `docs/02-gazebo-unity/unity-sim.mdx`:
    - Unity as alternative to Gazebo, Unity Robotics Hub, ROS-TCP-Connector
  - Create `docs/02-gazebo-unity/sensors-lidar.mdx`:
    - LiDAR sensor simulation, ray tracing, point cloud data
    - Code example: Gazebo LiDAR plugin configuration
  - Create `docs/02-gazebo-unity/sensors-depth.mdx`:
    - Depth camera simulation (RealSense D435i model), RGB-D data
    - Code example: Gazebo depth camera plugin
  - Create `docs/02-gazebo-unity/sensors-imu.mdx`:
    - IMU simulation, accelerometer, gyroscope, orientation
  - Create `docs/02-gazebo-unity/urdf-sdf.mdx`:
    - Comparison of URDF vs SDF (Simulation Description Format)
    - Converting between formats
  - **Test**: `yarn start`, navigate through all 7 chapters in Module 2 sidebar

### Module 3: AI-Robot Brain (NVIDIA Isaac) - 5 chapters + Latency Warnings

- [ ] **T009** [P] [US1] [US3] Create Module 3 content (docs/03-isaac/) with <LatencyWarning />
  - Create `docs/03-isaac/index.mdx` (Module 3 intro)
  - Create `docs/03-isaac/isaac-setup.mdx`:
    - NVIDIA Isaac Sim installation (Omniverse launcher, system requirements)
    - First launch, sample worlds
  - Create `docs/03-isaac/synthetic-data.mdx`:
    - Generating training data in simulation (randomized scenes, domain randomization)
    - Exporting labeled images, depth maps, segmentation masks
  - Create `docs/03-isaac/vslam.mdx`:
    - Visual SLAM algorithms (ORB-SLAM, RTAB-Map)
    - Running VSLAM in Isaac Sim
  - Create `docs/03-isaac/nav2.mdx`:
    - ROS 2 Nav2 navigation stack (costmaps, planners, controllers)
    - Integrating Nav2 with Isaac Sim
  - Create `docs/03-isaac/sim-to-real.mdx`:
    - **Import LatencyWarning**: `import LatencyWarning from '@site/src/components/LatencyWarning';`
    - **Section 1**: Sim-to-real workflow overview
    - **Section 2**: Training in the cloud (AWS g5.xlarge, local RTX workstation)
    - **Add <LatencyWarning />**: Place in first 2 paragraphs discussing cloud-edge communication
    - **Section 3**: Exporting weights (ONNX, TensorRT)
    - **Section 4**: Deploying to Jetson Orin Nano (SSH, SCP file transfer)
    - **Section 5**: Edge inference (<50ms latency requirement)
    - **Mermaid diagram**: Sim-to-real architecture flow
      ````markdown
      ```mermaid
      graph LR
          A[Cloud Training<br/>AWS g5.xlarge] --> B[Export Weights<br/>ONNX/TensorRT]
          B --> C[Deploy to Edge<br/>Jetson Orin Nano]
          C --> D[Edge Inference<br/><10ms latency]
          A -.X Cloud Control<br/>50-200ms latency<br/>UNSAFE!.-> E[Real Robot]
          D --> E
      ```
      ````
  - **Test**: `yarn start`, navigate to /docs/03-isaac/sim-to-real, verify `<LatencyWarning />` renders with yellow/orange background

### Module 4: Vision-Language-Action (VLA) - 4 chapters + Latency Warnings

- [ ] **T010** [P] [US1] [US3] Create Module 4 content (docs/04-vla/) with <LatencyWarning />
  - Create `docs/04-vla/index.mdx` (Module 4 intro)
  - Create `docs/04-vla/whisper.mdx`:
    - OpenAI Whisper model for speech-to-text
    - Installation (pip install openai-whisper)
    - Code example: Transcribing voice commands
  - Create `docs/04-vla/llm-planning.mdx`:
    - Using LLMs (GPT-4, Claude) for natural language planning
    - "Clean the room" example: Natural language → task breakdown → ROS actions
    - Example flow diagram:
      - User: "Clean the room"
      - Whisper: Transcribes to text
      - LLM: Plans subtasks ["Navigate to living room", "Detect obstacles", "Pick up objects", "Place in bin"]
      - ROS: Executes actions sequentially
  - Create `docs/04-vla/ros-integration.mdx`:
    - **Import LatencyWarning**: `import LatencyWarning from '@site/src/components/LatencyWarning';`
    - **Section 1**: Integrating VLA models with ROS 2 action servers
    - **Add <LatencyWarning />**: Place when discussing cloud LLM APIs (OpenAI, Anthropic) vs local models
    - **Section 2**: Edge deployment options (local LLM on Jetson vs cloud API)
    - **Section 3**: ROS action client calling LLM planning service
    - Code example: Python ROS node that calls LLM API and publishes actions
  - Create `docs/04-vla/capstone.mdx`:
    - **Capstone Project**: Autonomous Humanoid with Voice Commands
    - Project overview: Build a voice-controlled robot that can execute natural language commands
    - Architecture diagram (Mermaid):
      ````markdown
      ```mermaid
      graph TD
          A[Voice Input] --> B[Whisper Transcription]
          B --> C[LLM Planning<br/>GPT-4 / Claude]
          C --> D[ROS Action Server]
          D --> E[Robot Execution<br/>Jetson Orin Nano]
          E --> F[Feedback to User]
      ```
      ````
    - Implementation steps:
      1. Set up ROS 2 action server
      2. Integrate Whisper for speech-to-text
      3. Call LLM API for planning
      4. Execute actions on robot (simulated or real)
      5. Deploy to Jetson for edge inference
    - Deliverables: See Assessment 4 in course-overview.mdx
    - Grading rubric
  - **Test**: `yarn start`, navigate through all 4 chapters in Module 4, verify `<LatencyWarning />` in ros-integration.mdx

---

## Phase 4: Custom Styling & Assets (Tasks 11-12)

**Purpose**: Apply custom theme, add images/diagrams (User Story 3 - Interactive Reading)

- [ ] **T011** [P] [US3] Customize theme and add dark mode support
  - Edit `src/css/custom.css`:
    - Override Docusaurus theme colors for dark mode
    - Ensure `<LatencyWarning />` background colors work in both light and dark modes
    - Increase code block font size for readability
    - Style sidebar icons (🤖, 🏗️, 🧠, 🗣️)
  - Verify dark mode toggle works on all pages

- [ ] **T012** [P] [US2] [US3] Add hardware images and diagrams to docs/assets/
  - Create `docs/assets/` directory
  - Add images:
    - `jetson-orin-nano.jpg` (or link to NVIDIA official image URL)
    - `realsense-d435i.jpg` (or link to Intel official image URL)
    - `unitree-go2.jpg`
    - `unitree-g1.jpg`
  - Embed images in `docs/hardware.mdx`:
    ```markdown
    ![Jetson Orin Nano Developer Kit](./assets/jetson-orin-nano.jpg)
    ```
  - Add alt text for accessibility (WCAG 2.1 Level AA compliance)
  - Optimize images with `@docusaurus/plugin-ideal-image` (already in dependencies)

---

## Phase 5: Testing & Quality Assurance (Tasks 13-14)

**Purpose**: Validate content, accessibility, and production build (User Story 1-3)

- [ ] **T013** [US1] [US2] [US3] Manual content validation
  - Verify all internal links work (no 404s):
    - Click through all sidebar navigation items
    - Test links in intro.mdx, course-overview.mdx, hardware.mdx
  - Verify code examples are copy-pastable:
    - Test "Copy" button on at least 10 code blocks
    - Ensure syntax highlighting works (Python, XML, Bash)
  - Verify hardware prices accurate:
    - Check Jetson Orin Nano: $249 (NVIDIA Store)
    - Check RealSense D435i: $349 (Intel Store)
    - Check ReSpeaker: $69 (Seeed Studio)
    - Update "Last Updated" timestamp if prices changed
  - Verify Latency Trap warnings present:
    - Module 3: `docs/03-isaac/sim-to-real.mdx` (within first 2 paragraphs)
    - Module 4: `docs/04-vla/ros-integration.mdx` (when discussing cloud APIs)
  - Verify dark mode renders correctly:
    - Toggle dark mode on each page
    - Check `<LatencyWarning />` background colors
    - Check code block syntax highlighting
  - Verify mobile responsiveness:
    - Open Chrome DevTools Device Mode
    - Test on iPhone SE (375px), iPad (768px), Pixel 5 (393px)
    - Ensure sidebar collapses on mobile
    - Ensure tables scroll horizontally on narrow screens

- [ ] **T014** [US4] Build production site and verify output
  - Run: `yarn build`
  - Verify build succeeds (no errors)
  - Output directory: `build/`
  - Check build warnings (should be minimal)
  - Run: `yarn serve` (serve production build locally)
  - Open: http://localhost:3000
  - Verify navigation works in production build
  - Verify search works (if Algolia DocSearch configured, otherwise built-in search)
  - Check Lighthouse scores (run in Chrome DevTools):
    - Performance: >90
    - Accessibility: 100 (WCAG 2.1 Level AA)
    - Best Practices: >90
    - SEO: 100

---

## Phase 6: Deployment (Task 15)

**Purpose**: Set up GitHub Actions workflow for automatic GitHub Pages deployment (User Story 4)

- [ ] **T015** [US4] Setup GitHub Pages deployment workflow
  - Create `.github/workflows/deploy.yml`:
    ```yaml
    name: Deploy to GitHub Pages

    on:
      push:
        branches:
          - main
          - 001-docusaurus-textbook
      workflow_dispatch:

    permissions:
      contents: read
      pages: write
      id-token: write

    concurrency:
      group: "pages"
      cancel-in-progress: false

    jobs:
      build:
        runs-on: ubuntu-latest
        steps:
          - name: Checkout
            uses: actions/checkout@v4

          - name: Setup Node.js
            uses: actions/setup-node@v4
            with:
              node-version: '18'
              cache: 'yarn'

          - name: Install dependencies
            run: yarn install --frozen-lockfile

          - name: Build site
            run: yarn build

          - name: Setup Pages
            uses: actions/configure-pages@v4

          - name: Upload artifact
            uses: actions/upload-pages-artifact@v3
            with:
              path: ./build

      deploy:
        environment:
          name: github-pages
          url: ${{ steps.deployment.outputs.page_url }}
        runs-on: ubuntu-latest
        needs: build
        steps:
          - name: Deploy to GitHub Pages
            id: deployment
            uses: actions/deploy-pages@v4
    ```
  - Update `docusaurus.config.ts`:
    - Set `baseUrl: '/ai_robotics_book/'` (if repo name is `ai_robotics_book`)
    - Set `url: 'https://[USERNAME].github.io'`
    - Set `organizationName: '[USERNAME]'`
    - Set `projectName: 'ai_robotics_book'`
  - Update `README.md`:
    - Add GitHub Actions status badge: `![Deploy](https://github.com/[USERNAME]/ai_robotics_book/actions/workflows/deploy.yml/badge.svg)`
    - Add link to deployed site: `https://[USERNAME].github.io/ai_robotics_book/`
  - Create `LICENSE` file:
    - Content (CC-BY-SA 4.0 for docs, MIT for code):
      ```
      # License

      ## Content (Documentation, Tutorials, Educational Materials)
      Licensed under Creative Commons Attribution-ShareAlike 4.0 International (CC-BY-SA 4.0)
      https://creativecommons.org/licenses/by-sa/4.0/

      ## Code (Software, Scripts, Examples)
      Licensed under MIT License
      https://opensource.org/licenses/MIT

      See full license texts at the URLs above.
      ```
  - Update footer in `docusaurus.config.ts`:
    - Add copyright: `© 2025 Physical AI & Humanoid Robotics Textbook`
    - Add license links:
      ```typescript
      footer: {
        style: 'dark',
        copyright: `© 2025 Physical AI & Humanoid Robotics Textbook. Content: <a href="https://creativecommons.org/licenses/by-sa/4.0/" target="_blank">CC-BY-SA 4.0</a> | Code: <a href="https://opensource.org/licenses/MIT" target="_blank">MIT</a>`,
      }
      ```
  - **Commit and push**:
    ```bash
    git add .
    git commit -m "feat: complete Physical AI textbook with 4 modules, custom MDX components, and GitHub Pages deployment

    - Initialize Docusaurus v3 with TypeScript
    - Configure 4-module sidebar (ROS 2, Gazebo/Unity, Isaac, VLA)
    - Create custom MDX components (HardwareTable, LatencyWarning, PlatformNote)
    - Add intro, course overview, hardware requirements pages
    - Create 23 chapter files across 4 modules (Module 1: 7 chapters, Module 2: 7 chapters, Module 3: 5 chapters, Module 4: 4 chapters)
    - Include Latency Trap warnings in Module 3 (sim-to-real) and Module 4 (ros-integration)
    - Add $700 Economy Jetson Kit pricing table (Jetson Orin Nano $249, RealSense D435i $349, ReSpeaker $69)
    - Configure GitHub Actions workflow for automatic GitHub Pages deployment
    - Add CC-BY-SA 4.0 (content) + MIT (code) dual licensing

    🤖 Generated with Claude Code (https://claude.com/claude-code)

    Co-Authored-By: Claude <noreply@anthropic.com>"
    git push origin 001-docusaurus-textbook
    ```
  - **Configure GitHub Pages**:
    - Go to repository Settings → Pages
    - Source: Select "GitHub Actions"
    - Verify workflow runs successfully (check Actions tab)
    - Once deployed, visit `https://[USERNAME].github.io/ai_robotics_book/`
  - **Test deployment**:
    - Navigate to all 4 modules
    - Verify hardware table renders with $700 breakdown
    - Verify Latency Trap warnings visible in Module 3 & 4
    - Test dark mode toggle
    - Test mobile responsiveness
    - Test search functionality

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies - start immediately
  - Tasks 1-3 can run sequentially (T001 → T002 → T003)
  - **Checkpoint after T001**: Run `yarn install && yarn start` to verify Docusaurus setup

- **Phase 2 (Core Content)**: Depends on Phase 1 completion
  - Tasks 4-6 can run in parallel (all create different files in `docs/`)
  - **Test after Phase 2**: Run `yarn start`, verify intro, course-overview, hardware pages load

- **Phase 3 (Module Content)**: Depends on Phase 1-2 completion
  - Tasks 7-10 can run in parallel (each module is independent)
  - **Test after each module**: Run `yarn start`, navigate through module chapters

- **Phase 4 (Styling & Assets)**: Depends on Phase 3 completion
  - Tasks 11-12 can run in parallel (T011: CSS, T012: images)

- **Phase 5 (Testing)**: Depends on Phase 1-4 completion
  - Task 13: Manual validation (sequential)
  - Task 14: Production build (sequential, depends on T13 passing)

- **Phase 6 (Deployment)**: Depends on Phase 5 completion
  - Task 15: GitHub Actions setup (sequential)
  - **Final test**: `git push` triggers deployment, verify live site

### Parallel Opportunities

Within each phase, tasks marked **[P]** can run in parallel:
- **Phase 1**: T003 can start after T001 completes (independent of T002)
- **Phase 2**: T004, T005, T006 all parallel (different files)
- **Phase 3**: T007, T008, T009, T010 all parallel (different module directories)
- **Phase 4**: T011, T012 parallel (CSS vs images)

---

## Testing Strategy

### After Task 1 (T001):
```bash
yarn install
yarn start
```
- Verify localhost:3000 loads default Docusaurus site

### After Task 2 (T002):
```bash
yarn start
```
- Verify sidebar shows 4 module categories with icons (🤖, 🏗️, 🧠, 🗣️)

### After Task 5 (T005):
```bash
yarn start
```
- Navigate to http://localhost:3000/docs/hardware
- Verify Economy Jetson Kit table renders with $700 total

### After Task 7 (T007):
```bash
yarn start
```
- Navigate through Module 1 (ROS 2) chapters
- Verify code blocks have copy buttons
- Verify platform notes render

### After Task 9 (T009):
```bash
yarn start
```
- Navigate to /docs/03-isaac/sim-to-real
- Verify `<LatencyWarning />` renders with yellow/orange background
- Verify Mermaid diagram renders

### After Task 10 (T010):
```bash
yarn start
```
- Navigate to /docs/04-vla/ros-integration
- Verify `<LatencyWarning />` renders
- Navigate to /docs/04-vla/capstone
- Verify capstone architecture diagram renders

### After Task 14 (T014):
```bash
yarn build
yarn serve
```
- Open http://localhost:3000
- Verify production build works
- Run Lighthouse audit in Chrome DevTools

### After Task 15 (T015):
```bash
git add .
git commit -m "feat: complete Physical AI textbook"
git push origin 001-docusaurus-textbook
```
- Monitor GitHub Actions workflow in Actions tab
- Once deployed, visit https://[USERNAME].github.io/ai_robotics_book/
- Verify all content accessible publicly

---

## Notes

- **Constitution Compliance**: All tasks adhere to 11 constitutional principles (Single Source of Truth in /docs/, verified $700 prices, Latency Trap warnings, CC-BY-SA 4.0 + MIT licensing)
- **Avoid Over-Engineering**: Tasks focus on MVP delivery (4 modules, core content, deployment) without premature optimization
- **Code Reuse**: Custom MDX components (HardwareTable, LatencyWarning, PlatformNote) used across multiple chapters
- **Accessibility**: All tasks include WCAG 2.1 Level AA considerations (alt text, semantic HTML, keyboard navigation)
- **Version Control**: Commit after each major phase (after T003, after T006, after T010, after T012, after T014, final after T015)
- **Estimated Timeline**: 15-20 days with content authoring parallelized across 5 Claude Code Subagents (ros2-expert, isaac-sim-guide, vla-instructor, hardware-advisor, capstone-architect)
