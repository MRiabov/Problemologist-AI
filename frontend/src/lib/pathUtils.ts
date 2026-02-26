/**
 * Utility class for robust path manipulation in the browser.
 * Similar to Node.js 'path' or Python's 'pathlib'.
 */
export class PathUtils {
  /**
   * Joins multiple path segments into a single path, ensuring only one slash between them.
   * Handles leading/trailing slashes correctly.
   */
  static join(...segments: string[]): string {
    return segments
      .map((segment, index) => {
        if (!segment) return '';
        
        let cleaned = segment;
        // Remove leading slash except for the first segment
        if (index > 0) {
          cleaned = cleaned.replace(/^\/+/, '');
        }
        // Remove trailing slash except for the last segment
        if (index < segments.length - 1) {
          cleaned = cleaned.replace(/\/+$/, '');
        }
        return cleaned;
      })
      .filter(segment => segment !== '')
      .join('/');
  }

  /**
   * Ensures a path starts with a single leading slash.
   */
  static ensureLeadingSlash(path: string): string {
    if (!path) return '/';
    return '/' + path.replace(/^\/+/, '');
  }

  /**
   * Removes any leading slashes from a path.
   */
  static stripLeadingSlash(path: string): string {
    if (!path) return '';
    return path.replace(/^\/+/, '');
  }
}
