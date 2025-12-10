# Implementation Plan: Navbar Upgrade for AI-Native Textbook Website

**Branch**: `005-navbar-upgrade` | **Date**: 2025-12-10 | **Spec**: [specs/005-navbar-upgrade/spec.md](specs/005-navbar-upgrade/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The implementation plan focuses on upgrading the landing page navbar to feature only essential navigation elements: logo, "AI Native Development" link, "Book" link on the left side, with search bar and light/dark mode toggle on the right side. This involves modifying Docusaurus configuration to remove extra header links while preserving existing functionality for search and theme toggling. The approach leverages Docusaurus' built-in navbar configuration options with minimal CSS overrides for custom spacing and alignment.

## Technical Context

**Language/Version**: JavaScript/TypeScript, Docusaurus v2.x
**Primary Dependencies**: Docusaurus framework, React components
**Storage**: N/A (configuration-based changes)
**Testing**: Browser-based testing, responsive design validation
**Target Platform**: Web (all modern browsers)
**Project Type**: Static site generated with Docusaurus
**Performance Goals**: Maintain current page load times, no performance degradation
**Constraints**: Must preserve existing search functionality and theme toggle, mobile responsive
**Scale/Scope**: Single-page application (landing page navbar only)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The implementation follows the project constitution by:
- Using minimal, stable configuration changes
- Maintaining backward compatibility with existing functionality
- Following Docusaurus best practices
- Ensuring responsive design principles
- Preserving accessibility standards

## Project Structure

### Documentation (this feature)

```text
specs/005-navbar-upgrade/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
ai-book/
├── docusaurus.config.js     # Main configuration file to update
├── src/
│   └── css/
│       └── custom.css       # Custom CSS overrides if needed
└── static/
    └── img/                 # Logo images
```

**Structure Decision**: Single project structure with configuration changes to docusaurus.config.js and potential custom CSS for spacing adjustments. The navbar is managed through Docusaurus' built-in navigation system with minimal custom code.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Custom CSS overrides | Precise spacing control for navbar elements | Docusaurus default spacing doesn't meet design requirements |

## Architecture Sketch

The new navbar architecture will follow a split layout:
- **Left Side**: Logo → "AI Native Development" link → "Book" link
- **Right Side**: Search bar → Light/Dark mode toggle
- **Removed Elements**: All other header links, dropdowns, and navigation items
- **Responsive Behavior**: On mobile, left elements collapse into hamburger menu while search and theme toggle remain accessible

## Section Structure

### Navigation Logic
- Logo click navigates to homepage (`/`)
- "AI Native Development" link navigates to homepage (`/`)
- "Book" link navigates to documentation root (`/docs/intro`)
- Search functionality preserved as-is
- Theme toggle functionality preserved as-is

### Component Structure
- Left group: `Logo` + `AI Native Development` link + `Book` link
- Right group: `Search` component + `ThemeToggle` component
- Mobile behavior: Left group collapses into hamburger menu, right group remains visible

### Styling Rules
- Spacing: 24px between logo and first link, 16px between subsequent links
- Typography: Use existing Docusaurus font sizes and weights
- Alignment: Left group left-aligned, right group right-aligned
- Responsive breakpoints: Desktop (>1024px), Tablet (768px-1024px), Mobile (<768px)

### Mobile Hamburger Behavior
- Left-side elements collapse into mobile menu
- Search and theme toggle remain visible in navbar
- Hamburger icon appears on mobile to access collapsed menu

### Docusaurus Configuration Updates
- Modify `themeConfig.navbar.items` to include only required elements
- Update logo configuration if needed
- Preserve search plugin configuration
- Preserve theme toggle configuration

## Research Approach

### Docusaurus Navbar Validation
- Consult official Docusaurus documentation for navbar configuration options
- Review Docusaurus theme configuration API for navigation items
- Test configuration changes in local development environment
- Validate against multiple browsers and devices

### Stability and Compatibility
- Use documented Docusaurus configuration options rather than custom components
- Maintain existing plugin configurations for search and theme toggle
- Test across multiple screen sizes and browsers
- Ensure accessibility compliance is maintained

### Local Testing
- Run `npm start` to launch development server
- Test navbar functionality across different screen sizes
- Verify all links navigate to correct destinations
- Confirm search and theme toggle functionality remains intact

## Quality Validation

### Visual Consistency
- Alignment must stay consistent across screen sizes
- Logo displays correctly in both light and dark modes
- Spacing between elements follows design specifications
- Typography remains consistent with site's overall design

### Functional Validation
- "AI Native Development" link navigates to homepage
- "Book" link navigates to documentation root page
- Search bar opens command palette and functions properly
- Dark mode toggle switches themes correctly
- Removed links no longer appear in navbar

### Responsive Behavior
- Navbar renders correctly in mobile collapsed mode
- Hamburger menu appears and functions on mobile devices
- Search and theme toggle remain accessible on all screen sizes
- No horizontal scrolling on mobile devices

## Decisions Needing Documentation

### Logo Placement
- Decision: Logo positioned on the far left, followed by text links
- Rationale: Standard web navigation pattern, clear visual hierarchy

### Link Grouping
- Decision: Keep "AI Native Development" and "Book" as separate inline elements
- Rationale: Provides clear, distinct navigation options without grouping complexity

### CSS Approach
- Decision: Use minimal CSS overrides rather than custom theme components
- Rationale: Maintains compatibility with Docusaurus updates, reduces complexity

### Mobile Navigation
- Decision: Only left-side elements collapse into hamburger menu
- Rationale: Search and theme toggle are frequently used, should remain accessible

## Testing Strategy

### Development Testing
- Run `npm start` and inspect navbar across different breakpoints (320px, 768px, 1024px, 1440px)
- Verify clicking logo navigates to homepage
- Verify "Book" link navigates to docs root page
- Verify search bar opens command palette
- Verify dark mode toggle functions correctly
- Confirm removed links no longer appear in navbar

### Cross-Browser Testing
- Test in Chrome, Firefox, Safari, and Edge
- Verify consistent rendering and functionality
- Check for any browser-specific styling issues

### Mobile Testing
- Use browser device emulation for various mobile devices
- Test actual mobile devices if possible
- Verify hamburger menu functionality
- Confirm touch targets meet accessibility standards

## Expected Deliverables

### Configuration Changes
- Updated `docusaurus.config.js` with new navbar configuration
- Custom CSS in `src/css/custom.css` if spacing adjustments needed

### Final Navbar Structure
- Left side: Logo, "AI Native Development" link, "Book" link
- Right side: Search bar, theme toggle
- Mobile behavior: Collapsible menu for left-side elements

### Implementation Notes
- Clear documentation of changes made to `docusaurus.config.js`
- Comments explaining any custom CSS additions
- Instructions for future maintenance of navbar configuration