# Data Model: Book Layout and Structure

## Entities

### Book

Represents the entire book project.

**Attributes**:
-   `title`: (String) The main title of the book.
-   `author`: (String) The name of the book's author.
-   `subtitle`: (String, optional) A tagline or subtitle for the book.
-   `modules`: (List of Module) A collection of modules that constitute the book.

**Relationships**:
-   Contains multiple `Module` entities.

### Module

A top-level organizational unit within the book.

**Attributes**:
-   `title`: (String) The title of the module.
-   `summary`: (String) A brief summary paragraph for the module.
-   `chapters`: (List of Chapter) A collection of chapters within the module.

**Relationships**:
-   Belongs to one `Book`.
-   Contains multiple `Chapter` entities.

### Chapter

A division within a module.

**Attributes**:
-   `title`: (String) The title of the chapter.
-   `summary`: (String) A brief summary paragraph for the chapter.
-   `sections`: (List of Section) A collection of sections within the chapter.

**Relationships**:
-   Belongs to one `Module`.
-   Contains multiple `Section` entities.

### Section

A granular division of content within a chapter.

**Attributes**:
-   `title`: (String) The title of the section.
-   `introduction`: (String) An introductory paragraph for the section.
-   `subsections`: (List of Subsection, optional) An optional collection of subsections.
-   `key_takeaways`: (List of String) Bullet points summarizing the section's key learnings.

**Relationships**:
-   Belongs to one `Chapter`.
-   Can optionally contain multiple `Subsection` entities.

### Subsection

An optional, further division within a section.

**Attributes**:
-   `title`: (String) The title of the subsection.
-   `content`: (String) The main content of the subsection.

**Relationships**:
-   Belongs to one `Section`.
