/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { CodeReference } from './CodeReference';
import type { GeometricSelection } from './GeometricSelection';
export type SteerablePrompt_Input = {
    text: string;
    selections?: Array<GeometricSelection>;
    code_references?: Array<CodeReference>;
    mentions?: Array<string>;
};

