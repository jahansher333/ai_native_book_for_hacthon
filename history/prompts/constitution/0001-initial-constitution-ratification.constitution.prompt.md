---
id: 0001
title: Initial Constitution Ratification
stage: constitution
date: 2025-12-04
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: jahansher333
command: /sp.constitution
labels: ["constitution", "principles", "governance", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

# Project Constitution
**Hackathon I: Physical AI & Humanoid Robotics Textbook**
Built with Spec-Kit Plus + Claude Code Cli

These principles are **immutable**. Every file, commit, design decision, and AI-generated output in this repository **MUST** obey them. Any violation is grounds for immediate rejection by CI/CD and human reviewers.

1. **Single Source of Truth**
   The official course content lives ONLY in `/content/` as Markdown/MDX files inside a Docusaurus v3 project. No duplicate Google Docs, Notion pages, or scattered notes are allowed.

2. **Specification-First Development**
   Every chapter, feature, and component MUST have a machine-readable spec in `/specs/` (YAML + Markdown) before any implementation begins. Claude Code Subagents are forbidden from writing prose or code without an approved spec.

3. **Accuracy & Technical Truth**
   All technical claims about ROS 2, NVIDIA Isaac Sim, Jetson Orin Nano, Unitree G1/Go2, RealSense D435i, latency traps, sim-to-real transfer, VLA models, etc., must be 100% factually correct as of December 2025. Every hardware price and TOPS figure must match the Economy Jetson Kit table ($249 + $349 + $69 + $30 ≈ $700).

4. **Sim-to-Real First Philosophy**
   Every lab, tutorial, and capstone must explicitly teach:
   • Train in the cloud (AWS g5 or local RTX workstation)
   • Export weights
   • Deploy & run inference on Jetson Orin Nano/Orin NX
   Controlling a real robot from the cloud is forbidden in all examples.

5. **Cost Transparency**
   Every hardware recommendation must list exact current prices and include the $700 Economy Jetson Kit as the default student path. Cloud alternatives must show the ~$205/quarter calculation.

6. **Urdu + Personalization Ready**
   The Docusaurus site MUST be architected from day one to support:
   • Per-chapter "Translate to Urdu" button (using DeepL or custom NLLB model)
   • Per-chapter "Personalize for me" button that rewrites content based on user profile (hardware/software background collected at signup via Better-Auth)

7. **RAG Chatbot is First-Class Citizen**
   The embedded RAG chatbot (FastAPI + Neon Postgres + Qdrant Cloud + OpenAI Agents SDK) MUST:
   • Answer from the book only
   • Support selected-text-only queries
   • Be fully functional on the deployed GitHub Pages site

8. **Authentication & User Profiles (Better-Auth)**
   Signup/Signin via Better-Auth is mandatory for bonus points. At signup we collect:
   • Do you have an RTX GPU?
   • Do you own a Jetson?
   • Do you have access to a real robot?
   • Programming experience level
   These answers drive personalized chapter versions.

9. **Reusable Intelligence via Claude Code Subagents**
   All major sections (ROS 2, Isaac Sim, VLA, Hardware Guide, Capstone) MUST be generated or heavily assisted by dedicated Claude Code Subagents with their own Skills registered in `/agents/`.

10. **Open Source & Accessible Forever**
    License: CC-BY-SA 4.0 + MIT for code.
    Deployed permanently on GitHub Pages. No paywalls, no login required for reading (login only enhances experience).

11. **Latency Trap Rule**
    Any diagram, video, or tutorial showing robot control MUST clearly label whether it is simulation or real hardware, and explicitly warn when cloud control would be dangerous.

This constitution is the supreme law of the repository.
Any PR that violates it will be blocked by CODEOWNERS and the Spec-Kit Plus guardian agent.

Signed (virtually) on 4 December 2025
Project Lead: [Your Name Here]
Framework: Spec-Kit Plus + Claude Code
Goal: The world's best open-source Physical AI & Humanoid Robotics textbook.

## Response snapshot

Successfully created and ratified the Physical AI & Humanoid Robotics Textbook Constitution v1.0.0.

The constitution contains 11 immutable principles organized into the following categories:

**Content & Development Principles (I-III)**:
- Single Source of Truth (content in `/content/` only)
- Specification-First Development (specs in `/specs/` before implementation)
- Accuracy & Technical Truth (100% factual correctness with verified pricing)

**Educational Philosophy (IV-V)**:
- Sim-to-Real First Philosophy (cloud training → edge deployment)
- Cost Transparency (Economy Jetson Kit at $700 baseline)

**User Experience & Accessibility (VI-VIII)**:
- Urdu + Personalization Ready (i18n and adaptive content architecture)
- RAG Chatbot is First-Class Citizen (FastAPI + Qdrant + OpenAI Agents SDK)
- Authentication & User Profiles (Better-Auth with hardware/skill profiling)

**Development Infrastructure (IX-X)**:
- Reusable Intelligence via Claude Code Subagents (domain expert agents)
- Open Source & Accessible Forever (CC-BY-SA 4.0 + MIT licensing)

**Safety & Ethics (XI)**:
- Latency Trap Rule (explicit simulation vs. real hardware labeling + warnings)

**Governance**:
Established semantic versioning rules (MAJOR.MINOR.PATCH), amendment process requiring approval from project lead + 2 technical reviewers, and mandatory compliance reviews for all PRs.

**Template Alignment**:
- Reviewed plan-template.md (Constitution Check section validated)
- Reviewed spec-template.md (Requirements structure aligned)
- Reviewed tasks-template.md (Task categorization aligned)

All template placeholders replaced with concrete values. No ambiguities remain. Constitution is immediately enforceable.

## Outcome

- ✅ Impact: Created foundational governance document defining 11 immutable principles for Physical AI textbook project. Establishes architectural, technical, accessibility, and safety constraints that will guide all future development decisions.
- 🧪 Tests: none (governance document - no executable tests)
- 📁 Files: Created `.specify/memory/constitution.md` (291 lines, v1.0.0)
- 🔁 Next prompts: `/sp.specify` for first feature (e.g., Docusaurus site setup, RAG chatbot, Better-Auth integration), followed by `/sp.plan` and `/sp.tasks`
- 🧠 Reflection: Constitution successfully captures complex multi-dimensional requirements (technical accuracy, cost transparency, safety, accessibility, i18n). Enforcement mechanisms clearly defined with specific file paths and CI/CD checkpoints. Semantic versioning ensures controlled evolution.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): Constitution structure validated - PASS; All placeholders filled - PASS; Template consistency checked - PASS
- Prompt variant (if applicable): none (initial)
- Next experiment (smallest change to try): Monitor whether "Constitution Check" section in plan-template.md needs more specific guidance for the 11 principles (e.g., automated checks for pricing accuracy, spec file existence verification)
