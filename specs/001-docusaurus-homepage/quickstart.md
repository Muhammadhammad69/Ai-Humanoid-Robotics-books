# Quickstart Guide: Docusaurus Homepage Development

## Prerequisites

- Node.js v18 or higher
- npm or yarn package manager
- Git
- A code editor (VS Code recommended)

## Setup Instructions

### 1. Clone and Install Dependencies

```bash
# Clone the repository (if not already done)
git clone <repository-url>
cd <repository-name>

# Install dependencies
npm install
# OR
yarn install
```

### 2. Verify Docusaurus Installation

```bash
# Check if Docusaurus is properly installed
npx docusaurus --version

# Run the development server to verify setup
npm run start
# OR
yarn start
```

Your site should now be accessible at `http://localhost:3000`

### 3. Project Structure Overview

```
project-root/
├── docs/                 # Course content (preserved)
├── src/
│   ├── components/       # Custom React components
│   │   └── HomepageFeatures/
│   ├── pages/           # Page components
│   │   └── index.tsx    # Homepage
│   └── css/
│       └── custom.css   # Custom styles
├── static/              # Static assets
├── package.json         # Dependencies and scripts
├── docusaurus.config.js # Docusaurus configuration
└── sidebars.ts          # Navigation configuration
```

## Development Workflow

### 1. Clean Up Default Content

First, clean up existing homepage content as specified:

```bash
# Navigate to the source directory
cd src

# Remove existing HomepageFeatures component (if exists)
rm -rf components/HomepageFeatures/

# Clear the existing homepage content
# (We'll replace it with our new implementation)
```

### 2. Create Custom Homepage

Create the new homepage with the required sections:

```bash
# Create the homepage file if it doesn't exist
touch src/pages/index.tsx
```

### 3. Create Module Cards Component

```bash
# Create the directory structure for the component
mkdir -p src/components/HomepageFeatures
touch src/components/HomepageFeatures/index.tsx
```

### 4. Update Custom CSS

```bash
# Ensure the custom CSS file exists
touch src/css/custom.css
```

## Running the Development Server

```bash
# Start the development server
npm run start
# OR
yarn start

# Build for production
npm run build
# OR
yarn build

# Serve production build locally for testing
npm run serve
# OR
yarn serve
```

## Key Configuration Files

### Docusaurus Configuration
Edit `docusaurus.config.js` to update:
- Site title and description
- Theme configuration
- Navbar and footer settings

### Custom CSS Variables
In `src/css/custom.css`, define the color palette:

```css
:root {
  --ifm-color-primary: #1A73E8;
  --ifm-color-secondary: #39FF14;
  --ifm-color-accent: #8B5CF6;
  --ifm-background-color: #F5F7FA;
  --ifm-font-color-base: #2E2E2E;
}

html[data-theme='dark'] {
  --ifm-background-color: #0A0A0A;
}
```

## Testing Checklist

Before deployment, verify:

- [ ] Homepage loads correctly on desktop
- [ ] Responsive design works on tablet and mobile
- [ ] All module cards display with correct icons and descriptions
- [ ] Color palette matches specifications
- [ ] Hover effects work properly
- [ ] Call-to-action button is visible and functional
- [ ] All sections are accessible
- [ ] No console errors
- [ ] Page loads within 3 seconds

## Common Commands

```bash
# Clean build cache
npm run clear
# OR
yarn clear

# Check for broken links
npm run swizzle
# OR
yarn swizzle

# Generate static HTML for deployment
npm run build
# OR
yarn build
```

## Troubleshooting

### If the site doesn't load:
1. Verify all dependencies are installed: `npm install`
2. Clear cache: `npm run clear`
3. Restart the development server: `npm run start`

### If custom styles don't appear:
1. Verify `src/css/custom.css` is properly linked in the Docusaurus config
2. Check that CSS variables are correctly defined
3. Ensure there are no CSS conflicts with the default theme