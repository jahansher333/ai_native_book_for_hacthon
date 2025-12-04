---
id: 0003
title: Textbook Task List Generation
stage: tasks
date: 2025-12-04
surface: agent
model: claude-opus-4-5-20251101
feature: 001-docusaurus-textbook
branch: 001-docusaurus-textbook
user: jahansher333
command: /sp.tasks
labels: ["tasks", "implementation", "docusaurus", "content-authoring", "mdx-components", "github-actions"]
links:
  spec: specs/001-docusaurus-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-docusaurus-textbook/tasks.md
tests:
 - yarn install && yarn start (after T001)
 - yarn build (after T014)
 - git push (after T015 - triggers GitHub Pages deployment)
---

## Prompt

/sp.tasks
For spec 001-book-creation and plan.md:
Break into 15 ordered tasks grouped by user story. Include [P] for parallel (e.g., write multiple module intros). File paths exact: e.g., Task 1: npx create-docusaurus@latest . --typescript (docusaurus.config.ts). Task 5: Create docs/hardware.mdx with $700 table MDX. Task 10: Add latency warnings to sim sections. Dependencies: Setup before content. Tests: yarn start preview after each module; yarn build for deploy. End with Task 15: Setup gh-pages workflow.

## Response snapshot

Successfully created comprehensive task list with exactly 15 ordered tasks for Physical AI & Humanoid Robotics Textbook implementation on branch `001-docusaurus-textbook`.

**Task Organization**:

**Phase 1: Setup & Foundation (Tasks 1-3)**
- T001: Initialize Docusaurus v3 with TypeScript (`npx create-docusaurus@latest`, configure `docusaurus.config.ts` with theme, plugins, baseUrl)
- T002: Configure `sidebars.ts` with 4-module structure (🤖 ROS 2, 🏗️ Gazebo/Unity, 🧠 Isaac, 🗣️ VLA) - 23 chapters total
- T003 [P]: Create custom MDX components (`HardwareTable/`, `LatencyWarning/`, `PlatformNote/`, `BonusFeaturePlaceholder/`)

**Phase 2: Core Content Pages (Tasks 4-6)**
- T004: Create `docs/intro.mdx` (Why Physical AI Matters, 6 learning outcomes, prerequisites)
- T005: Create `docs/hardware.mdx` with `<HardwareTable variant="economy" />` ($700 breakdown: Jetson $249, RealSense $349, ReSpeaker $69, peripherals ~$100)
- T006: Create `docs/course-overview.mdx` (13-week breakdown, 4 assessments with deliverables)

**Phase 3: Module Content (Tasks 7-10) - Parallelizable**
- T007 [P]: Module 1 content (`docs/01-ros2/` - 7 chapters: index, nodes, topics, services, actions, rclpy, urdf)
- T008 [P]: Module 2 content (`docs/02-gazebo-unity/` - 7 chapters: index, physics-sim, gazebo-setup, unity-sim, sensors-lidar, sensors-depth, sensors-imu, urdf-sdf)
- T009 [P]: Module 3 content (`docs/03-isaac/` - 5 chapters: index, isaac-setup, synthetic-data, vslam, nav2, **sim-to-real.mdx with `<LatencyWarning />`** + Mermaid diagram)
- T010 [P]: Module 4 content (`docs/04-vla/` - 4 chapters: index, whisper, llm-planning, **ros-integration.mdx with `<LatencyWarning />`**, capstone with architecture diagram)

**Phase 4: Custom Styling & Assets (Tasks 11-12)**
- T011 [P]: Customize `src/css/custom.css` (dark mode support, `<LatencyWarning />` colors, code block styling)
- T012 [P]: Add hardware images to `docs/assets/` (jetson-orin-nano.jpg, realsense-d435i.jpg, unitree robots)

**Phase 5: Testing & Quality Assurance (Tasks 13-14)**
- T013: Manual content validation (all links work, code copy-pastable, hardware prices accurate, Latency warnings present in Modules 3 & 4, dark mode works, mobile responsive)
- T014: Build production site (`yarn build`), verify output, run Lighthouse (Performance >90, Accessibility 100, SEO 100)

**Phase 6: Deployment (Task 15)**
- T015: Setup `.github/workflows/deploy.yml` (GitHub Actions: Checkout → Setup Node → Install → Build → Configure Pages → Upload → Deploy), update `docusaurus.config.ts` baseUrl, create LICENSE (CC-BY-SA 4.0 + MIT), update README with badge, commit and push

**Key Features**:
- **23 chapter files** across 4 modules (Module 1: 7, Module 2: 7, Module 3: 5, Module 4: 4)
- **Custom MDX components**: `<HardwareTable variant="economy" />`, `<LatencyWarning latencyRange="50-200ms+" />`, `<PlatformNote platforms={[...]} />`
- **Latency Trap warnings**: Module 3 `sim-to-real.mdx` (first 2 paragraphs), Module 4 `ros-integration.mdx` (cloud API section)
- **Mermaid diagrams**: Sim-to-real architecture flow, VLA capstone architecture
- **$700 Economy Jetson Kit**: Exact pricing table with vendor links (Jetson Orin Nano $249, RealSense D435i $349, ReSpeaker $69, peripherals ~$100)
- **13-week course breakdown**: Weeks 1-13 with topics, 4 assessments with deliverables and grading criteria
- **GitHub Pages deployment**: Automatic via Actions on push to main/branch

**Testing Strategy**:
- After T001: `yarn install && yarn start` (verify Docusaurus loads)
- After T005: Verify hardware table renders with $700 total
- After T007-T010: Preview each module (`yarn start`)
- After T009: Verify `<LatencyWarning />` in Module 3 sim-to-real
- After T010: Verify `<LatencyWarning />` in Module 4 ros-integration
- After T014: `yarn build && yarn serve` (test production build, run Lighthouse)
- After T015: `git push` (trigger deployment, verify live site)

**Parallelization**:
- Phase 1: T003 can start after T001 (independent of T002)
- Phase 2: T004, T005, T006 all parallel (different files)
- Phase 3: T007, T008, T009, T010 all parallel (different module directories) - **KEY PARALLELIZATION OPPORTUNITY** for 5 Claude Code Subagents
- Phase 4: T011, T012 parallel (CSS vs images)

**Constitution Compliance**:
- All content in `/docs/` (Principle I)
- $700 exact prices verified (Principle V)
- Latency Trap warnings in Modules 3 & 4 (Principle XI)
- CC-BY-SA 4.0 + MIT dual licensing (Principle X)
- No cloud robot control examples (Principle IV)

## Outcome

- ✅ Impact: Created executable task list with exactly 15 tasks organized into 6 phases for Physical AI & Humanoid Robotics Textbook implementation. Tasks cover complete workflow from Docusaurus initialization to GitHub Pages deployment, including 23 chapter files, custom MDX components (HardwareTable, LatencyWarning, PlatformNote), $700 Economy Jetson Kit pricing table, Latency Trap warnings in Modules 3 & 4, and automated CI/CD. Enables parallelized content authoring (Tasks 7-10) across 5 Claude Code Subagents for 15-20 day delivery.
- 🧪 Tests: Defined at each checkpoint - `yarn start` after T001/T005/T007-T010 (preview), `yarn build` after T014 (production validation), `git push` after T015 (deployment trigger), Lighthouse audit (Performance >90, Accessibility 100) in T014
- 📁 Files: Created `specs/001-docusaurus-textbook/tasks.md` (580+ lines covering 15 tasks with exact file paths, testing strategy, parallelization opportunities, dependencies, constitution compliance validation)
- 🔁 Next prompts: `/sp.implement` to execute all 15 tasks, or execute tasks sequentially starting with T001 (`npx create-docusaurus@latest`), then T002-T015
- 🧠 Reflection: Task breakdown successfully balances granularity (15 tasks, not too many) with completeness (covers all 4 user stories, all 11 constitutional principles). Parallelization strategy (Tasks 7-10) critical for timeline - enables 4 modules to be authored simultaneously by dedicated subagents. Testing checkpoints after each phase ensure early detection of issues. Task 15 (GitHub Actions deployment) as final task ensures smooth production release.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Task count validation - PASS (exactly 15 tasks as requested); User story coverage - PASS (all 4 stories US1-US4 covered); File path specificity - PASS (exact paths like `docs/01-ros2/nodes.mdx`, `src/components/HardwareTable/index.tsx`); Testing strategy - PASS (checkpoints after T001, T005, T007-T010, T014, T015); Parallelization markers - PASS ([P] tags on T003, T004-T006, T007-T010, T011-T012)
- Prompt variant (if applicable): none (initial)
- Next experiment (smallest change to try): Consider splitting T007-T010 (module content creation) into 2 tasks each: (1) Create placeholder MDX files with front matter and section headings, (2) Fill content. This would enable earlier validation of sidebar structure and reduce merge conflicts when multiple subagents work simultaneously. Trade-off: Increases task count to 19 tasks (exceeds requested 15).
