// The web APIs the page's logic modules use, which Node provides too. Declared here so that
// tsconfig.logic.json checks those modules without the DOM, as Node runs them.
declare class URL {
  constructor(url: string, base?: string);
  readonly href: string;
  readonly origin: string;
}

declare class TextDecoder {
  constructor(label?: string);
  decode(input?: ArrayBuffer | ArrayBufferView): string;
}
