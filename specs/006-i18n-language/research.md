# Research: Docusaurus Internationalization (i18n) Implementation for Urdu Language

## Overview
Research conducted to implement internationalization support for Urdu language in the AI-Humanoid Robotics educational website using Docusaurus 3.9.2's native i18n system.

## Decision: Use Docusaurus Native i18n System
**Rationale**: Docusaurus provides a robust, built-in i18n system that handles RTL languages, SEO, and file-based translation workflow efficiently. This approach aligns with the project's technology stack and ensures compatibility with existing TypeScript configuration.

**Alternatives considered**:
1. Third-party i18n libraries (i18next, react-i18next) - Rejected due to potential conflicts with Docusaurus's built-in system and added complexity
2. Custom translation solution - Rejected due to maintenance overhead and lack of RTL support
3. Separate Urdu-specific website - Rejected due to content duplication and maintenance challenges

## Configuration in docusaurus.config.ts
**Decision**: Update i18n configuration to include Urdu locale with RTL support
**Rationale**: Following Docusaurus documentation, this approach ensures proper RTL rendering and language switching functionality.

```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    ur: {
      label: 'اردو',
      direction: 'rtl',  // Critical for Urdu RTL support
      htmlLang: 'ur',
      path: 'ur',
    },
  },
},
```

## Directory Structure for Translations
**Decision**: Use Docusaurus plugin-based directory structure in i18n/ur/
**Rationale**: This follows Docusaurus conventions and ensures proper integration with the build system.

```
i18n/
└── ur/
    ├── code.json                    # Custom component translations
    ├── docusaurus-plugin-content-docs/
    │   └── current/                 # Documentation files in Urdu
    │       ├── intro.md
    │       ├── preface.md
    │       ├── glossary.md
    │       ├── appendices.md
    │       ├── hardware/
    │       └── modules/
    │           ├── module-1/
    │           ├── module-2/
    │           ├── module-3/
    │           └── module-4/
    ├── docusaurus-plugin-content-pages/
    │   └── index.module.css         # Homepage translations
    └── docusaurus-theme-classic/
        ├── footer.json              # Footer translations
        └── navbar.json              # Navbar translations
```

## RTL (Right-to-Left) Language Support
**Decision**: Use Docusaurus built-in RTL support via `direction: 'rtl'` property
**Rationale**: Docusaurus automatically handles CSS adjustments for RTL languages, including proper text alignment and navigation flow while keeping code blocks LTR.

**Key considerations**:
- Text flows from right to left
- Navigation elements align to the right
- Code blocks remain LTR within RTL context
- Proper rendering of Arabic script used in Urdu
- Responsive design maintains integrity in RTL mode

## CLI Commands for Translation Management
**Decision**: Implement npm scripts for translation workflow management
**Rationale**: Provides standardized workflow for translation creation, development, and build processes.

```json
"scripts": {
  "write-translations-ur": "docusaurus write-translations --locale ur",
  "start:ur": "docusaurus start --locale ur",
  "build:ur": "docusaurus build --locale ur",
  "build:all": "docusaurus build --locale en,ur"
}
```

## Custom Component Translation Strategy
**Decision**: Use @docusaurus/Translate component and translate() function for custom components
**Rationale**: Ensures consistent translation handling across custom React components while maintaining TypeScript compatibility.

**Implementation approach**:
- Use `<Translate>` component for JSX elements
- Use `translate()` function for string translations in JavaScript/TypeScript
- Extract component strings to `i18n/ur/code.json`
- Use `useDocusaurusContext()` hook for site metadata access

## Build Process Considerations
**Decision**: Leverage Docusaurus's automatic locale-specific build generation
**Rationale**: Docusaurus creates distinct standalone applications for each locale with proper SEO and routing.

**Key aspects**:
- Default locale (English) omits locale name in URL
- Non-default locales (Urdu) get `/ur/` path segment automatically
- Separate bundles generated for each locale
- Automatic `hreflang` header generation for SEO
- Build time increase should remain under 20% (requirement)

## Language Switching Mechanism
**Decision**: Implement locale dropdown in navbar for language switching
**Rationale**: Provides intuitive user interface for language selection while maintaining current page context.

```typescript
navbar: {
  items: [
    {
      type: 'localeDropdown',
      position: 'right',
    },
    // ... other navbar items
  ],
},
```

**Features**:
- Preserves current page context when switching languages
- Stores user preferences in localStorage
- Automatic URL path updates with locale prefix
- Proper SEO with hreflang tags

## Translation Workflow
**Decision**: Use incremental translation approach prioritizing high-traffic content
**Rationale**: Allows for phased implementation while ensuring core functionality is available early.

**Priority order**:
1. Configuration files (docusaurus.config.ts, package.json)
2. UI elements (navbar, footer, sidebar labels)
3. High-traffic documentation (intro, preface, module landing pages)
4. Custom components (homepage, HeroSection, HomepageFeatures)
5. Module content (chapters, exercises, code samples)
6. Hardware documentation

## Technical Constraints and Solutions
**Constraint**: Preserve technical accuracy of code samples
**Solution**: Keep code samples and technical terms untranslated while translating explanatory text

**Constraint**: Maintain existing TypeScript configuration
**Solution**: Update configuration with i18n settings without modifying other TypeScript settings

**Constraint**: Support RTL layout for Urdu
**Solution**: Use Docusaurus built-in RTL support with `direction: 'rtl'` property

**Constraint**: Ensure SEO compliance
**Solution**: Leverage Docusaurus automatic hreflang tag generation

## References
- Docusaurus i18n documentation: https://docusaurus.io/docs/i18n/introduction
- Docusaurus configuration API: https://docusaurus.io/docs/api/docusaurus-config#i18n
- Docusaurus CLI commands: https://docusaurus.io/docs/cli#docusaurus-write-translations