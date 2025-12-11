# Data Model: Docusaurus Homepage Design

## Overview
This document describes the data structures and component interfaces for the Physical AI & Humanoid Robotics homepage.

## Component Data Structures

### Homepage Data
The main homepage doesn't require a backend data model since it's a static page, but it follows this conceptual structure:

```
Homepage
├── HeroSection
│   ├── title: string (e.g., "Physical AI & Humanoid Robotics")
│   ├── tagline: string (e.g., "Mastering embodied intelligence")
│   ├── backgroundImage: string (gradient or image path)
│   └── ctaButton: CTAButton
├── CourseOverview
│   ├── title: string
│   └── description: string
├── ModuleCards: ModuleCard[]
├── LearningOutcomes
│   ├── title: string
│   └── outcomes: string[]
├── WhyPhysicalAIMatters
│   ├── title: string
│   └── content: string
├── HardwareRequirements
│   ├── title: string
│   └── requirements: HardwareItem[]
└── CallToAction
    └── ctaButton: CTAButton
```

### ModuleCard
Represents one of the four main course modules:

```
ModuleCard
├── id: string (unique identifier)
├── title: string (e.g., "The Robotic Nervous System (ROS 2)")
├── icon: string (emoji or icon reference)
├── description: string
├── link: string (URL to detailed module page)
└── priority: number (1-4 for ordering)
```

### CTAButton
Call-to-action button component:

```
CTAButton
├── text: string (e.g., "Start Learning")
├── link: string (URL to next step)
├── style: ButtonStyle
└── trackingId: string (for analytics)
```

### ButtonStyle
Visual style options for buttons:

```
ButtonStyle
├── variant: "primary" | "secondary" | "outline"
├── size: "small" | "medium" | "large"
├── color: string (CSS color value)
└── glowEffect: boolean
```

### HardwareItem
Represents a hardware requirement:

```
HardwareItem
├── id: string (unique identifier)
├── name: string (e.g., "Development Workstation")
├── icon: string (emoji or icon reference)
├── description: string
└── detailsLink: string (URL to detailed requirements)
```

## Content Structure

### Color Palette
The design follows a specific color scheme:

```
ColorPalette
├── primary: "#1A73E8" (Electric Blue)
├── secondary: "#39FF14" (Neon Green)
├── accent: "#8B5CF6" (Violet Radiance)
├── background: "#F5F7FA" (Soft White)
├── darkBackground: "#0A0A0A" (Midnight Black)
├── textColor: "#2E2E2E" (Graphite Gray)
└── cardBackground: "white with soft shadow"
```

### Typography
Font sizing and styling:

```
Typography
├── heroHeading: { size: "48-64px", weight: "bold", family: "system default" }
├── sectionHeading: { size: "32-40px", weight: "bold", family: "system default" }
├── bodyText: { size: "16-18px", weight: "normal", family: "system default" }
└── linkStyle: { underline: "neon accent on hover" }
```

## Responsive Layout Structure

### Grid Layouts
```
GridLayout
├── moduleCards
│   ├── desktop: { columns: 2, rows: 2, gap: "2rem" }
│   ├── tablet: { columns: 2, rows: 2, gap: "1.5rem" }
│   └── mobile: { columns: 1, rows: 4, gap: "1rem" }
├── learningOutcomes
│   ├── desktop: { columns: 2, gap: "2rem" }
│   └── mobile: { columns: 1, gap: "1rem" }
└── hardwareRequirements
    ├── desktop: { columns: 3, gap: "1.5rem" }
    └── mobile: { columns: 1, gap: "1rem" }
```

## Component States

### ModuleCard States
```
ModuleCardState
├── default: { glow: "none", transform: "none" }
├── hover: { glow: "neon border", transform: "lift effect" }
└── active: { glow: "stronger neon", transform: "slight scale" }
```

### Theme States
```
ThemeState
├── light: { background: "#F5F7FA", text: "#2E2E2E" }
└── dark: { background: "#0A0A0A", text: "#FFFFFF" }
```