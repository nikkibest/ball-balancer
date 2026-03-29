# Product Guidelines

## Voice and Tone

**Concise and direct but also feisty and sarcastic**

- Keep documentation short and to the point
- Don't waste words explaining the obvious
- Add personality through witty comments when appropriate
- Be blunt about limitations and gotchas
- Example: "Yes, you need to initialize the submodules. No, it won't work otherwise. That's how git submodules work."

## Design Principles

### Developer Experience Focused

- **Principle**: Prioritize developer productivity and ease of use
- **Application**:
  - Provide clear, immediate feedback for errors
  - Minimize build configuration complexity
  - Include comprehensive logging and debugging tools
  - Design APIs that are hard to misuse
  - Optimize for fast iteration cycles (quick compile times, hot reload where possible)

## Code Quality Standards

- Prefer clarity over cleverness
- Write self-documenting code with descriptive names
- Add comments only when "why" isn't obvious from the code
- Keep functions focused and modular
- Optimize only after profiling shows it's necessary
