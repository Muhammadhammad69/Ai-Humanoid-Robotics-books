# Data Model: Internationalization (i18n) Support for Urdu Language

## Overview
Data model for the AI-Humanoid Robotics educational website's internationalization system, focusing on the Urdu language implementation.

## Key Entities

### 1. Language Preference
**Description**: User's selected language preference with persistence mechanism
**Attributes**:
- locale: string (e.g., 'en', 'ur')
- persistence: localStorage
- duration: minimum 30 days
- context: current page preservation across language switches

**Relationships**:
- Associated with user session
- Maps to URL locale prefixes
- Connects to translation file lookup

### 2. Translation Files
**Description**: Organized structure of Urdu content mirroring English documentation
**Attributes**:
- path: string (mirrors English content structure)
- format: JSON for UI strings, markdown for documentation
- direction: 'rtl' for Urdu, 'ltr' for English
- fallback: graceful degradation to English for missing translations

**Relationships**:
- Maps to English source content
- Connects to Docusaurus plugin structure
- Associated with locale-specific builds

### 3. RTL Layout Configuration
**Description**: Text direction configuration that affects page rendering for Urdu
**Attributes**:
- direction: string ('rtl' or 'ltr')
- cssOverrides: object (RTL-specific CSS adjustments)
- elementAlignment: object (navigation, text flow, UI positioning)

**Relationships**:
- Connected to locale configuration
- Affects all page rendering for Urdu locale
- Maintains LTR for code blocks within RTL context

### 4. Language Switching Mechanism
**Description**: System that handles transitions between languages while preserving context
**Attributes**:
- currentPage: string (URL path to maintain context)
- localePrefix: string ('' for English, '/ur/' for Urdu)
- transitionTime: number (<2 seconds reload time)
- persistenceMethod: localStorage

**Relationships**:
- Connects to navbar locale dropdown
- Maps to URL routing system
- Associated with build output structure

## Validation Rules

### Language Preference Validation
- Must be one of supported locales (en, ur)
- Must persist across browser sessions
- Must maintain current page context during switch
- Must update URL with appropriate locale prefix

### Translation File Validation
- Must follow Docusaurus plugin directory structure
- Must maintain markdown formatting and frontmatter
- Must preserve code blocks and technical terms
- Must provide valid JSON format for UI strings
- Must gracefully fallback to English when translation missing

### RTL Layout Validation
- Must apply right-to-left text direction for Urdu
- Must keep code blocks left-to-right within RTL pages
- Must maintain responsive design integrity
- Must preserve visual hierarchy and readability

### Language Switching Validation
- Must preserve current page context during switch
- Must update URL with correct locale prefix
- Must complete transition in under 2 seconds
- Must maintain user's selection across sessions

## State Transitions

### Language Selection Flow
1. User accesses language dropdown
2. System presents available languages (English, اردو)
3. User selects Urdu
4. System applies RTL layout and updates content
5. System updates URL to include /ur/ prefix
6. System stores preference in localStorage

### Translation Resolution Flow
1. System identifies requested locale
2. System looks up translation files in i18n/[locale]/
3. If translation exists, use translated content
4. If translation missing, fallback to English content
5. Apply appropriate text direction (RTL for Urdu)

## API Contracts (Conceptual)
While this is primarily a static site with file-based i18n, the following conceptual interfaces define the translation system:

```
GET /api/translations/{locale}/{path}
Response: {
  "content": "translated content",
  "fallback": boolean,
  "direction": "rtl|ltr"
}

POST /api/preferences/language
Request: {
  "locale": "ur|en",
  "preserveContext": string (current path)
}
Response: {
  "status": "success",
  "redirectUrl": string
}
```