/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
/**
 * A single COTS part returned from search.
 */
export type CotsSearchItem = {
    part_id: string;
    name: string;
    category: string;
    manufacturer: string;
    price: number;
    source: string;
    weight_g: number;
    metadata_vars?: Record<string, any>;
};

