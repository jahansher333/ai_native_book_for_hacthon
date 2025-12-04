---
id: 0002
title: Textbook Implementation Plan
stage: plan
date: 2025-12-04
surface: agent
model: claude-opus-4-5-20251101
feature: 001-docusaurus-textbook
branch: 001-docusaurus-textbook
user: jahansher333
command: /sp.plan
labels: ["planning", "architecture", "docusaurus", "mdx", "github-pages", "physical-ai"]
links:
  spec: specs/001-docusaurus-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-docusaurus-textbook/plan.md
tests:
 - none
---

## Prompt

/sp.plan
For spec 001-book-creation:
Tech Stack: Docusaurus v3 with TypeScript, MDX v2 for interactive code/tables, @docusaurus/theme-classic, preset-classic, plugin-content-docs. Use sidebars.js for 4-module navigation. Static generation only—no backend. Integrate remark-math for equations, rehype-raw for custom HTML. Hardware table as reusable MDX component. Deploy via .github/workflows/gh-pages.yml (actions/configure-pages, actions/upload-pages-artifact).
Architecture: /docs/intro.mdx (overview) + /docs/{01-ros2,02-gazebo-unity,03-isaac,04-vla} folders with _category_.json. Embed Economy Jetson table in hardware.mdx. Sim-to-real diagrams via Mermaid. Research: Latest Docusaurus deploy to GitHub Pages (2025 updates). Constraints: Obey constitution—$700 kit exact, latency warnings, Urdu/personalization stubs (i18n plugin).
Milestones: 1. Init Docusaurus. 2. Structure modules. 3. Add content from course details. 4. Deploy workflow.

## Response snapshot

Successfully created comprehensive implementation plan for Physical AI & Humanoid Robotics Textbook on branch `001-docusaurus-textbook`.

**Plan Structure**:

**Summary**: Build static Docusaurus v3 site with 4 modules, custom MDX components, GitHub Pages deployment

**Technical Context**:
- **Stack**: TypeScript 5.x, MDX v2, Node.js 18.x LTS
- **Dependencies**: @docusaurus/preset-classic ^3.5.0, @docusaurus/theme-mermaid, remark-math + rehype-katex, rehype-raw
- **Project Type**: Static documentation site (no backend)
- **Performance Goals**: <3s page load, <500ms navigation, <1s search
- **Scale**: 4 modules × ~10 chapters = ~40 pages, ~200 code snippets, ~20 hardware components

**Constitution Check**: ✅ ALL 11 PRINCIPLES SATISFIED
- Principle I (Single Source of Truth): Content in `/docs/` only
- Principle III (Accuracy): Hardware prices verified Dec 2025
- Principle IV (Sim-to-Real First): No cloud robot control examples
- Principle V (Cost Transparency): $700 Economy Jetson Kit ($249 + $349 + $69 + ~$100)
- Principle X (Open Source): CC-BY-SA 4.0 + MIT licensing
- Principle XI (Latency Trap Rule): `<LatencyWarning />` MDX component in Modules 3 & 4

**Project Structure**:
```
ai_robotics_book/
├── docs/                            # Content (Constitution Principle I)
│   ├── intro.mdx                    # "Why Physical AI Matters"
│   ├── course-overview.mdx          # 13-week breakdown + 4 assessments
│   ├── hardware.mdx                 # Economy Jetson Kit + alternatives
│   ├── 01-ros2/                     # Module 1 (7 chapters)
│   ├── 02-gazebo-unity/             # Module 2 (7 chapters)
│   ├── 03-isaac/                    # Module 3 (5 chapters, includes sim-to-real.mdx)
│   └── 04-vla/                      # Module 4 (4 chapters, includes capstone.mdx)
├── src/components/                  # Custom MDX components
│   ├── HardwareTable/               # Economy Jetson Kit pricing table
│   ├── LatencyWarning/              # Yellow/orange warning callout
│   ├── PlatformNote/                # Code example platform compatibility
│   └── BonusFeaturePlaceholder/     # "Sign up to unlock" stubs
├── .github/workflows/deploy.yml     # GitHub Actions → GitHub Pages
├── docusaurus.config.ts             # Site configuration (TypeScript)
├── sidebars.ts                      # 4-module sidebar structure
└── LICENSE                          # CC-BY-SA 4.0 + MIT
```

**Phase 0: Research** (1 day)
- Docusaurus 3.5 best practices (Dec 2025)
- GitHub Pages deployment with Actions v4 (actions/configure-pages@v4, actions/upload-pages-artifact@v3, actions/deploy-pages@v4)
- MDX v2 component patterns (custom `<HardwareTable />`, `<LatencyWarning />`)
- Mermaid integration for sim-to-real diagrams
- i18n plugin stub for Urdu translation (placeholder only)
- Algolia DocSearch vs built-in search

**Phase 1: Design** (1 day)
- Data models: Module, Chapter, Hardware Component, Hardware Configuration, Assessment, Week
- MDX component contracts:
  - `<HardwareTable variant="economy" />`: Props, usage, rendered output (table with $700 breakdown)
  - `<LatencyWarning latencyRange="50-200ms+" />`: Yellow/orange callout for cloud control dangers
  - `<PlatformNote platforms={['Linux','macOS','WSL2']} />`: Code example compatibility
- Quickstart guide: `npm install` → `npm start` → local dev at localhost:3000

**Phase 2: Implementation Milestones** (15-20 days)
1. **Milestone 1: Initialize Docusaurus** (1-2 days)
   - Install Docusaurus 3.5 + TypeScript
   - Configure docusaurus.config.ts (metadata, dark mode, baseUrl for GitHub Pages)
   - Create sidebars.ts with 4-module placeholders
   - Add LICENSE (CC-BY-SA 4.0 + MIT)

2. **Milestone 2: Content Structure & Custom Components** (2-3 days)
   - Create `/docs/` structure with 4 module folders
   - Add `_category_.json` files (module labels, icons, positions)
   - Build custom MDX components (HardwareTable, LatencyWarning, PlatformNote, BonusFeaturePlaceholder)
   - Create intro.mdx, course-overview.mdx, hardware.mdx

3. **Milestone 3: Module Content Authoring** (10-15 days, parallelizable)
   - **Strategy**: Deploy 5 Claude Code Subagents (ros2-expert, isaac-sim-guide, vla-instructor, hardware-advisor, capstone-architect)
   - Module 1: 7 chapters (nodes, topics, services, actions, rclpy, urdf)
   - Module 2: 7 chapters (physics-sim, gazebo-setup, unity-sim, sensors)
   - Module 3: 5 chapters (isaac-setup, synthetic-data, vslam, nav2, sim-to-real with `<LatencyWarning />`)
   - Module 4: 4 chapters (whisper, llm-planning, ros-integration with `<LatencyWarning />`, capstone)
   - All code snippets with `<PlatformNote />`, hardware prices verified

4. **Milestone 4: GitHub Pages Deployment** (1 day)
   - Create .github/workflows/deploy.yml
   - Workflow: Checkout → Setup Node → Install deps → Build → Configure Pages → Upload artifact → Deploy
   - Update docusaurus.config.ts with correct baseUrl
   - Verify deployment at https://[USERNAME].github.io/ai_robotics_book/

5. **Milestone 5: Quality Assurance & Launch** (2-3 days)
   - Manual validation: All links work, code examples copy-pastable, prices verified
   - Lighthouse CI: Performance >90, Accessibility 100, SEO 100
   - Cross-browser testing (Chrome, Firefox, Safari, Edge)
   - Mobile responsiveness (320px-768px)
   - Constitution compliance final check (all 11 principles)

**Key Technical Decisions**:
- **Docusaurus over alternatives** (Nextra, VitePress, MkDocs): Best documentation UX, React/MDX ecosystem, GitHub Pages integration, Meta-backed
- **Static site over dynamic** (Next.js, Remix): <3s page loads, free GitHub Pages hosting, no backend = no vulnerabilities
- **Algolia DocSearch over built-in search**: <1s search results, better UX; fallback to built-in if rejected
- **MDX v2 over v3**: Docusaurus 3.5 officially supports MDX v2 (v3 experimental)

**NFR Mapping**:
- NFR-001 (<3s load): Static generation + prefetching + image optimization
- NFR-003 (<1s search): Algolia DocSearch
- NFR-004 (WCAG 2.1 AA): Semantic HTML + ARIA + keyboard nav + Lighthouse CI
- NFR-007 (Content in /docs/): Enforce via file structure + code review

**Risks & Mitigation**:
- Hardware prices change → Add "Last Updated" timestamp + quarterly verification
- GitHub Pages deployment fails → Status badge + email notifications + previous build remains live
- Content authoring delays → Parallelize across 5 subagents
- Accessibility violations → Lighthouse CI on every PR + manual axe DevTools audit

The plan is ready for task generation via `/sp.tasks`.

## Outcome

- ✅ Impact: Created production-ready implementation plan for Physical AI & Humanoid Robotics Textbook with complete technical architecture (Docusaurus v3 + TypeScript + MDX v2), 5-milestone roadmap (15-20 days), custom MDX component contracts (HardwareTable, LatencyWarning), GitHub Pages deployment workflow, and all 11 constitutional principles validated. Defines parallelizable content authoring strategy using 5 Claude Code Subagents (ros2-expert, isaac-sim-guide, vla-instructor, hardware-advisor, capstone-architect).
- 🧪 Tests: none (planning phase - implementation tests defined in plan: Lighthouse CI for performance/accessibility, manual content validation, cross-browser testing)
- 📁 Files: Created `specs/001-docusaurus-textbook/plan.md` (680+ lines covering Summary, Technical Context, Constitution Check, Project Structure, Phase 0-2, NFR Mapping, Risks, Technical Decisions)
- 🔁 Next prompts: `/sp.tasks` to generate executable task list organized by user story and milestone (will create ~50-80 tasks based on 5 milestones), then begin Phase 0 research
- 🧠 Reflection: Plan successfully balances technical rigor (TypeScript, MDX v2, Mermaid diagrams) with constitutional constraints (Single Source of Truth in /docs/, $700 Economy Jetson Kit pricing, Latency Trap warnings, open-source licensing). Custom MDX components (`<HardwareTable />`, `<LatencyWarning />`) elegantly solve interactive content requirements without backend complexity. Parallelized content authoring (5 subagents) critical to meet 15-20 day timeline for 40+ pages.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Constitution Check - ALL 11 PRINCIPLES PASS; NFR Mapping - ALL 10 NFRs COVERED; Project Structure - VALID (docs/ + src/components/ + .github/workflows/)
- Prompt variant (if applicable): none (initial)
- Next experiment (smallest change to try): Consider adding a Phase 1.5 "Content Scaffold" milestone between Milestone 2 (Structure) and Milestone 3 (Full Content) to generate placeholder MDX files with front matter and section headings before full content authoring. This would allow early validation of sidebar structure and enable subagents to work on separate chapters without merge conflicts.
