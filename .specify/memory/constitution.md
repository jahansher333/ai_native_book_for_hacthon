<!--
SYNC IMPACT REPORT
=====================
Version Change: NEW → 1.0.0
Constitution Type: MAJOR - Initial ratification
Modified Principles: None (initial creation)
Added Sections: All 11 principles + Governance
Removed Sections: None

Templates Status:
✅ .specify/templates/plan-template.md - Reviewed (Constitution Check section present)
✅ .specify/templates/spec-template.md - Reviewed (Requirements alignment confirmed)
✅ .specify/templates/tasks-template.md - Reviewed (Task structure alignment confirmed)
⚠ .specify/templates/commands/*.md - No command files found (expected for initial setup)

Follow-up TODOs: None

Date: 2025-12-04
Agent: claude-opus-4-5-20251101
-->

# Physical AI & Humanoid Robotics Textbook Constitution

**Hackathon I: Physical AI & Humanoid Robotics Textbook**
Built with Spec-Kit Plus + Claude Code CLI

These principles are **immutable**. Every file, commit, design decision, and AI-generated output in this repository **MUST** obey them. Any violation is grounds for immediate rejection by CI/CD and human reviewers.

## Core Principles

### I. Single Source of Truth

The official course content lives ONLY in `/content/` as Markdown/MDX files inside a Docusaurus v3 project. No duplicate Google Docs, Notion pages, or scattered notes are allowed.

**Rationale**: Multiple content sources create version conflicts, outdated information, and maintenance nightmares. A single source guarantees consistency and makes content updates deterministic.

**Enforcement**:
- All content authoring MUST occur in `/content/` directory
- External references MUST be imported/converted to Markdown
- CI/CD MUST fail if content exists outside designated paths
- Documentation links MUST point to repository files, not external documents

### II. Specification-First Development

Every chapter, feature, and component MUST have a machine-readable spec in `/specs/` (YAML + Markdown) before any implementation begins. Claude Code Subagents are forbidden from writing prose or code without an approved spec.

**Rationale**: Specifications establish clear contracts, enable automated validation, and prevent scope creep. Machine-readable specs allow tooling to verify compliance.

**Enforcement**:
- Implementation work MUST reference a spec file in `/specs/<feature>/spec.md`
- Pull requests without spec reference MUST be rejected
- Specs MUST be reviewed and approved before implementation starts
- Subagents MUST validate spec existence before generating content

### III. Accuracy & Technical Truth

All technical claims about ROS 2, NVIDIA Isaac Sim, Jetson Orin Nano, Unitree G1/Go2, RealSense D435i, latency traps, sim-to-real transfer, VLA models, etc., MUST be 100% factually correct as of December 2025. Every hardware price and TOPS figure MUST match the Economy Jetson Kit table ($249 + $349 + $69 + $30 ≈ $700).

**Rationale**: Incorrect technical information destroys credibility and wastes students' time and money. Precision in specifications and pricing enables informed decision-making.

**Enforcement**:
- Technical claims MUST cite authoritative sources (vendor docs, research papers, official specs)
- Hardware prices MUST be verified against current vendor listings
- Performance metrics (TOPS, latency, bandwidth) MUST be sourced from official specifications
- Outdated information MUST trigger automated warnings in CI
- Reviewers MUST verify all technical assertions before approval

**Economy Jetson Kit Standard**:
- Jetson Orin Nano Developer Kit: $249 (10 TOPS INT8)
- RealSense D435i: $349
- Power supply & peripherals: ~$100
- **Total**: ~$700 (verified December 2025)

### IV. Sim-to-Real First Philosophy

Every lab, tutorial, and capstone MUST explicitly teach:
- Train in the cloud (AWS g5 or local RTX workstation)
- Export weights
- Deploy & run inference on Jetson Orin Nano/Orin NX

Controlling a real robot from the cloud is **forbidden** in all examples.

**Rationale**: Cloud-controlled robots introduce catastrophic latency (50-200ms+), masking real-world constraints students MUST understand. Sim-to-real workflow mirrors industry best practices and teaches deployment discipline.

**Enforcement**:
- All tutorials MUST include explicit "Training Environment" and "Deployment Target" sections
- Code examples MUST demonstrate model export and edge deployment
- Architecture diagrams MUST show clear separation of training (cloud/workstation) and inference (edge device)
- Any cloud-robot control example MUST be rejected in review

**Workflow Template**:
1. **Train**: AWS g5.xlarge (NVIDIA A10G, 24GB VRAM) or local RTX 3060+
2. **Export**: ONNX/TensorRT format optimized for Jetson
3. **Deploy**: Transfer to Jetson via SSH/SCP
4. **Inference**: Run locally on Jetson with <50ms latency

### V. Cost Transparency

Every hardware recommendation MUST list exact current prices and include the $700 Economy Jetson Kit as the default student path. Cloud alternatives MUST show the ~$205/quarter calculation.

**Rationale**: Students need accurate budgeting information. Hidden costs destroy trust and create accessibility barriers.

**Enforcement**:
- Hardware sections MUST display pricing table with verified vendor links
- Cloud cost estimates MUST show hourly/monthly/quarterly breakdowns
- Comparisons MUST be fair (e.g., RTX 3060 vs. cloud GPU with equivalent VRAM)
- Prices MUST be updated quarterly via automated checks

**Verified Pricing (December 2025)**:
| Component | Price | Source |
|-----------|-------|--------|
| Jetson Orin Nano Dev Kit | $249 | NVIDIA Store |
| RealSense D435i | $349 | Intel Store |
| Power + cables + SD card | $100 | Amazon |
| **Economy Kit Total** | **$700** | - |
| AWS g5.xlarge (A10G) | $1.006/hr | AWS Pricing |
| **Cloud Alternative** | **~$205/quarter** | 5 hrs/week × 10 weeks |

### VI. Urdu + Personalization Ready

The Docusaurus site MUST be architected from day one to support:
- Per-chapter "Translate to Urdu" button (using DeepL or custom NLLB model)
- Per-chapter "Personalize for me" button that rewrites content based on user profile (hardware/software background collected at signup via Better-Auth)

**Rationale**: Accessibility and personalization are not optional features—they MUST be baked into architecture from the start to avoid expensive retrofits.

**Enforcement**:
- MDX components MUST support i18n infrastructure
- Content structure MUST separate translatable text from code/diagrams
- API endpoints for translation and personalization MUST be defined in `/specs/`
- UI components MUST include translation/personalization button placeholders
- User profile schema MUST be defined before chapter authoring begins

**Required Profile Fields**:
- Hardware access: `[RTX_GPU, JETSON, REAL_ROBOT, NONE]`
- Programming experience: `[BEGINNER, INTERMEDIATE, ADVANCED]`
- Preferred learning style: `[THEORY_FIRST, CODE_FIRST, PROJECT_BASED]`

### VII. RAG Chatbot is First-Class Citizen

The embedded RAG chatbot (FastAPI + Neon Postgres + Qdrant Cloud + OpenAI Agents SDK) MUST:
- Answer from the book only
- Support selected-text-only queries
- Be fully functional on the deployed GitHub Pages site

**Rationale**: Modern technical documentation requires intelligent search. RAG chatbot is not a "nice-to-have" but a core navigation interface.

**Enforcement**:
- Chatbot API MUST be deployed before site launch
- Content ingestion pipeline MUST run on every content update
- Chatbot responses MUST cite specific chapter/section sources
- Selection-based queries MUST include context of highlighted text
- GitHub Pages deployment MUST include chatbot integration

**Architecture Requirements**:
```
FastAPI Backend (deployed on Render/Railway)
   ↓
Neon Postgres (user sessions, query logs)
   ↓
Qdrant Cloud (vector embeddings of all chapters)
   ↓
OpenAI Agents SDK (query routing + response generation)
```

**Response Format MUST Include**:
- Answer text
- Source chapter + section link
- Confidence score
- Related chapters (if applicable)

### VIII. Authentication & User Profiles (Better-Auth)

Signup/Signin via Better-Auth is mandatory for bonus points. At signup we collect:
- Do you have an RTX GPU?
- Do you own a Jetson?
- Do you have access to a real robot?
- Programming experience level

These answers drive personalized chapter versions.

**Rationale**: Anonymous users get generic content. Authenticated users get content tailored to their hardware and skill level, dramatically improving learning outcomes.

**Enforcement**:
- Better-Auth MUST be integrated before personalization features
- Profile schema MUST match personalization requirements (Principle VI)
- Authentication MUST NOT block basic reading (login enhances, doesn't gate)
- Profile data MUST persist across sessions
- Privacy policy MUST disclose profile data usage

**User Flow**:
1. **Anonymous**: Read all content, use basic chatbot
2. **Sign Up**: Collect profile via 4-question form (~30 seconds)
3. **Authenticated**: Unlock "Personalize for me", save progress, tailored recommendations

### IX. Reusable Intelligence via Claude Code Subagents

All major sections (ROS 2, Isaac Sim, VLA, Hardware Guide, Capstone) MUST be generated or heavily assisted by dedicated Claude Code Subagents with their own Skills registered in `/agents/`.

**Rationale**: Subagents encode domain expertise and ensure consistency. Manual authoring of 500+ pages invites errors and inconsistency.

**Enforcement**:
- Each major topic MUST have a registered subagent in `/agents/<topic>/`
- Subagents MUST follow specs in `/specs/<topic>/spec.md`
- Generated content MUST be reviewed by human subject matter experts
- Subagent prompts MUST be version-controlled alongside code

**Required Subagents**:
1. **ros2-expert**: ROS 2 concepts, pub/sub, services, actions
2. **isaac-sim-guide**: Isaac Sim setup, scene authoring, sim-to-real
3. **vla-instructor**: Vision-Language-Action models, RT-1/RT-2/Octo
4. **hardware-advisor**: Jetson specs, RealSense, robot platforms
5. **capstone-architect**: Multi-week project design and scaffolding

### X. Open Source & Accessible Forever

License: CC-BY-SA 4.0 + MIT for code.
Deployed permanently on GitHub Pages. No paywalls, no login required for reading (login only enhances experience).

**Rationale**: Knowledge should be free. Open source ensures permanence, community contributions, and global accessibility.

**Enforcement**:
- LICENSE file MUST be present at repository root (CC-BY-SA 4.0 for content, MIT for code)
- All dependencies MUST be compatible with open-source licensing
- GitHub Pages deployment MUST be public
- No premium/gated content tiers allowed
- Community contributions MUST be accepted via standard PR process

**License Summary**:
```
Content (Markdown/MDX): CC-BY-SA 4.0
Code (Python/JS/C++): MIT
Assets (images/diagrams): CC-BY-SA 4.0 unless otherwise noted
```

### XI. Latency Trap Rule

Any diagram, video, or tutorial showing robot control MUST clearly label whether it is simulation or real hardware, and explicitly warn when cloud control would be dangerous.

**Rationale**: Cloud-controlled physical robots can cause injury or damage due to latency. Students MUST understand this risk immediately.

**Enforcement**:
- All architecture diagrams MUST include "SIMULATION" or "REAL HARDWARE" labels
- Tutorials involving cloud communication MUST display prominent latency warnings
- Video demos MUST show on-screen overlay indicating environment type
- Code comments MUST warn if network calls occur in control loops

**Required Warning Text** (for cloud control examples):
```
⚠️ LATENCY TRAP WARNING ⚠️
This architecture sends commands over the network.
Network latency (50-200ms+) makes this UNSAFE for real robots.
Use this pattern ONLY for:
- Simulation environments (Isaac Sim, Gazebo)
- High-level planning (not real-time control)
- Data collection (not actuation)

For real robots: Deploy models to edge devices (Jetson) for <10ms inference.
```

## Governance

This constitution supersedes all other practices, guidelines, and preferences. Amendments require:
1. Explicit documentation of proposed change
2. Approval from project lead and at least two technical reviewers
3. Migration plan for affected code/content
4. Version increment following semantic versioning rules

**Amendment Process**:
- **MAJOR** version: Backward-incompatible principle changes (e.g., removing a principle, contradicting existing rules)
- **MINOR** version: New principle additions or material expansions
- **PATCH** version: Clarifications, typo fixes, non-semantic refinements

**Compliance Review**:
- All pull requests MUST verify compliance via checklist
- CI/CD MUST enforce constitution checks (pricing accuracy, spec existence, license headers)
- Quarterly constitution audits MUST be conducted
- Violations MUST be documented in PR review comments with specific principle citations

**Constitution Authority**:
- In conflicts between this constitution and other documentation, constitution wins
- In conflicts between principles, project lead arbitrates with documented reasoning
- Complexity additions MUST be justified in `/specs/<feature>/plan.md` Complexity Tracking section

**Runtime Guidance**:
- See `CLAUDE.md` for AI assistant-specific development workflows
- See `.specify/templates/` for spec, plan, and task templates
- See `/specs/` for approved feature specifications

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
