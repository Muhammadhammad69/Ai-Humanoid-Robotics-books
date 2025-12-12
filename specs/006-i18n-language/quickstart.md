# Quickstart Guide: Internationalization (i18n) Support for Urdu Language

## Overview
This guide provides quick setup instructions for the AI-Humanoid Robotics educational website's Urdu language internationalization feature.

## Prerequisites
- Node.js 18+ installed
- Docusaurus 3.9.2 (already configured in project)
- Basic knowledge of TypeScript and React
- Understanding of Docusaurus configuration

## Setup Steps

### 1. Update Configuration
Update `docusaurus.config.ts` with i18n settings:

```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    ur: {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur',
      path: 'ur',
    },
  },
},
```

Add locale dropdown to navbar items:
```typescript
navbar: {
  items: [
    {
      type: 'localeDropdown',
      position: 'right',
    },
    // ... other items
  ],
},
```

### 2. Add Translation Scripts
Add to `package.json`:
```json
"scripts": {
  "write-translations-ur": "docusaurus write-translations --locale ur",
  "start:ur": "docusaurus start --locale ur",
  "build:ur": "docusaurus build --locale ur",
  "build:all": "docusaurus build --locale en,ur"
}
```

### 3. Generate Translation Files
Run the following command to generate initial translation structure:
```bash
npm run write-translations-ur
```

This creates the `i18n/ur/` directory with initial JSON files.

### 4. Translate UI Elements
Update the generated JSON files in `i18n/ur/`:
- `docusaurus-theme-classic/navbar.json` - Navbar labels
- `docusaurus-theme-classic/footer.json` - Footer text
- `docusaurus-plugin-content-docs/current.json` - Sidebar labels
- `code.json` - Custom component strings

### 5. Translate Documentation
Create Urdu versions of documentation files:
- `i18n/ur/docusaurus-plugin-content-docs/current/intro.md`
- `i18n/ur/docusaurus-plugin-content-docs/current/preface.md`
- Module landing pages and chapters
- Hardware documentation

### 6. Update Custom Components
Modify React components to support translations:
- Use `<Translate>` component for JSX elements
- Use `translate()` function for string translations
- Extract strings to `i18n/ur/code.json`

### 7. Test Implementation
```bash
# Start development server with Urdu locale
npm run start:ur

# Or start with both locales
npm run start

# Build for production
npm run build:all
```

## Key Features

### Language Switching
- Language dropdown in navbar
- Preserves current page context
- Stores preference in localStorage
- URL updates with locale prefix

### RTL Support
- Automatic right-to-left text direction for Urdu
- Code blocks maintain left-to-right direction
- Responsive design preserved
- Proper Arabic script rendering

### Fallback Mechanism
- Graceful fallback to English for untranslated content
- No broken pages if translation missing
- SEO-friendly with proper hreflang tags

## Common Tasks

### Adding New Translations
1. Add new markdown files to `i18n/ur/docusaurus-plugin-content-docs/current/`
2. Update sidebar navigation if needed
3. Test with `npm run start:ur`

### Updating Component Strings
1. Extract strings from components to `i18n/ur/code.json`
2. Use Docusaurus `@docusaurus/Translate` component
3. Verify translations render correctly

### Testing Language Switch
1. Navigate to any page
2. Select Urdu from language dropdown
3. Verify RTL layout and content
4. Test navigation within Urdu locale

## Troubleshooting

### Translation Not Showing
- Verify file exists in correct `i18n/ur/` subdirectory
- Check file format and syntax
- Ensure proper locale configuration in `docusaurus.config.ts`

### RTL Layout Issues
- Check `direction: 'rtl'` in locale config
- Verify CSS overrides in `src/css/custom.css` if needed
- Test code blocks maintain LTR direction

### Build Errors
- Run `npm run write-translations-ur` to regenerate translation files
- Verify all JSON files have valid syntax
- Check for missing dependencies