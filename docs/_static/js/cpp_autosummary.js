document.addEventListener('DOMContentLoaded', function() {
    // Clean up Exhale-generated link text while preserving HTML structure
    // TODO: Feels a little hacky, fix later
    const links = document.querySelectorAll('.rst-content section ul li a');
    
    links.forEach(link => {
        // Find the .std-ref span inside the link
        const stdRefSpan = link.querySelector('.std-ref');
        
        if (stdRefSpan) {
            const originalText = stdRefSpan.textContent.trim();
            
            // Remove verbose prefixes
            let cleanText = originalText
                .replace(/^(Class|Struct|Function|Typedef|Variable|Enum)\s+/, '')
                //.replace(/^ouster::[^:]*::([^(]+).*/, '$1'); // Extract just the name
                
            // Only update if we actually cleaned something
            if (cleanText !== originalText && cleanText.length > 0) {
                // Update only the text content of the .std-ref span
                stdRefSpan.textContent = cleanText;
            }
        } else {
            // Fallback for links without .std-ref spans
            const text = link.textContent.trim();
            let cleanText = text
                .replace(/^(Class|Struct|Function|Typedef|Variable|Enum)\s+/, '')
                //.replace(/^ouster::[^:]*::([^(]+).*/, '$1');
                
            if (cleanText !== text && cleanText.length > 0) {
                // Create a .std-ref span with the clean text
                const span = document.createElement('span');
                span.className = 'std std-ref';
                span.textContent = cleanText;
                
                // Replace the link content with the new span
                link.innerHTML = '';
                link.appendChild(span);
            }
        }
    });
});