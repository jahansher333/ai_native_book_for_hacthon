import re


def strip_mdx_syntax(text: str) -> str:
    """
    Remove JSX/MDX syntax while preserving markdown content

    Args:
        text: Raw MDX file content

    Returns:
        Clean markdown text without JSX components
    """
    # Remove MDX imports
    text = re.sub(r'^import\s+.*?;$', '', text, flags=re.MULTILINE)

    # Remove MDX exports
    text = re.sub(r'^export\s+.*$', '', text, flags=re.MULTILINE)

    # Remove JSX component opening/closing tags but keep content
    # <Component prop="value">content</Component> -> content
    text = re.sub(r'<[A-Z]\w*[^>]*>', '', text)
    text = re.sub(r'</[A-Z]\w*>', '', text)

    # Remove self-closing components
    # <Component prop="value" />
    text = re.sub(r'<[A-Z]\w*[^>]*/>', '', text)

    # Remove script tags completely
    text = re.sub(r'<script[^>]*>.*?</script>', '', text, flags=re.DOTALL)
    text = re.sub(r'<style[^>]*>.*?</style>', '', text, flags=re.DOTALL)

    # Remove MDX variable expressions ${...}
    text = re.sub(r'\$\{[^}]*\}', '', text)

    # Clean up extra whitespace
    text = re.sub(r'\n\n+', '\n\n', text)

    return text.strip()
