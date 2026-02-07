/* generated using openapi-typescript-codegen -- do not edit */
/* istanbul ignore file */
/* tslint:disable */
/* eslint-disable */
import type { Skill } from '../models/Skill';
import type { CancelablePromise } from '../core/CancelablePromise';
import { OpenAPI } from '../core/OpenAPI';
import { request as __request } from '../core/request';
export class SkillsService {
    /**
     * List Skills
     * List available skills.
     * @returns Skill Successful Response
     * @throws ApiError
     */
    public static listSkillsSkillsGet(): CancelablePromise<Array<Skill>> {
        return __request(OpenAPI, {
            method: 'GET',
            url: '/skills/',
        });
    }
}
